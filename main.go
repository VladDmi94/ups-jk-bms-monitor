package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"net/url"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
	"tinygo.org/x/bluetooth"
)

const (
	frameSize      = 300              // размер кадра JK BMS
	maxRxBuffer    = 4096             // защита от разрастания буфера приёма
	dischargeLimit = -0.1             // ток ниже этого значения (А) = работа от батареи
	debounceTicks  = 3                // сколько секунд подряд должно держаться новое состояние питания
	staleAfter     = 15 * time.Second // после этого данные считаются устаревшими
	kickAfter      = 10 * time.Second // если данных нет — повторно «будим» стрим BMS
	kickTimeout    = 5 * time.Second  // максимум на одну команду «будоражения» (защита от зависшего await)
	reconnectAfter = 30 * time.Second // если данных нет — переподключаемся
	minRetry       = 5 * time.Second
	maxRetry       = 60 * time.Second
	maxLogSize     = 1 << 20 // 1 МБ, затем лог ротируется в *.old
)

var (
	serviceUUID, _      = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	charUUID, _         = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
	headerResponseMagic = []byte{0x55, 0xAA, 0xEB, 0x90}
	adapter             = bluetooth.DefaultAdapter
	adapterReady        bool // используется только из горутины bmsLoop
)

type Config struct {
	TargetBMSName       string
	BatteryCapacityAh   float64
	DelayNotify         float64
	ShutdownThreshold   uint16
	LowBatteryThreshold uint16 // порог SOC (%) для предупреждения о низком заряде
	ShutdownTimeout     int
	StaleShutdown       int     // секунд без связи с BMS на батарее до аварийного выключения (0 = выкл.)
	TempThreshold       float64 // порог температуры (°C) для предупреждения в Telegram
	TempHysteresis      float64 // откат (°C) ниже порога, после которого предупреждение может повториться
}

// ---------- Логирование ----------

var (
	logMu   sync.Mutex
	logPath = "ups_log.txt"
)

func writeLog(format string, args ...any) {
	entry := fmt.Sprintf("%s - %s\n", time.Now().Format("02.01.2006 15:04:05"), fmt.Sprintf(format, args...))
	fmt.Print(entry)

	logMu.Lock()
	defer logMu.Unlock()

	if st, err := os.Stat(logPath); err == nil && st.Size() > maxLogSize {
		_ = os.Rename(logPath, logPath+".old")
	}
	f, err := os.OpenFile(logPath, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		fmt.Printf("Ошибка записи в лог: %v\n", err)
		return
	}
	defer f.Close()
	_, _ = f.WriteString(entry)
}

func fatal(format string, args ...any) {
	writeLog(format, args...)
	os.Exit(1)
}

// ---------- Конфигурация ----------

// baseDir — папка с .env: рядом с exe (важно для служб/планировщика, где cwd = System32), иначе текущая.
func baseDir() string {
	if exe, err := os.Executable(); err == nil {
		dir := filepath.Dir(exe)
		if _, err := os.Stat(filepath.Join(dir, ".env")); err == nil {
			return dir
		}
	}
	return "."
}

func loadEnv(filePath string) map[string]string {
	env := make(map[string]string)
	file, err := os.Open(filePath)
	if err != nil {
		return env
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := strings.TrimSpace(strings.TrimPrefix(scanner.Text(), "\ufeff")) // BOM от Блокнота
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		key, val, ok := strings.Cut(line, "=")
		if !ok {
			continue
		}
		val = strings.TrimSpace(val)
		if len(val) >= 2 && (val[0] == '"' || val[0] == '\'') && val[len(val)-1] == val[0] {
			val = val[1 : len(val)-1]
		}
		env[strings.TrimSpace(key)] = val
	}
	if err := scanner.Err(); err != nil {
		fmt.Printf("Ошибка чтения файла окружения: %v\n", err)
	}
	return env
}

// envNum возвращает число из .env; если ключа нет или значение некорректно (< min) — значение по умолчанию.
func envNum(env map[string]string, key string, def, min float64) float64 {
	raw := env[key]
	if raw == "" {
		return def
	}
	v, err := strconv.ParseFloat(raw, 64)
	if err != nil || v < min {
		writeLog("Предупреждение: некорректное значение %s=%q, используется %v", key, raw, def)
		return def
	}
	return v
}

// ---------- Состояние BMS ----------

type Telemetry struct {
	SOC             uint16
	Voltage         float64 // напряжение сборки, В
	Current         float64 // мгновенный ток (А): минус = разряд, плюс = заряд
	SmoothedCurrent float64 // EMA-сглаженный ток (А), для расчёта оставшегося времени
	Mos             float64 // температура MOS, °C
	T1              float64 // температура датчика T1, °C
	T2              float64 // температура датчика T2, °C
	Valid           bool
	Updated         time.Time
}

type BMSState struct {
	mu sync.Mutex
	t  Telemetry
}

// emaAlpha — коэффициент сглаживания тока (0..1). Чем меньше, тем плавнее.
// 0.1 = вес нового значения 10%, за ~10 с подстраивается под реальное изменение.
const emaAlpha = 0.1

func (s *BMSState) update(soc uint16, voltage, current, mos, t1, t2 float64) {
	s.mu.Lock()
	// EMA: новое сглаженное = alpha * мгновенное + (1 - alpha) * предыдущее сглаженное
	smoothed := s.t.SmoothedCurrent
	if !s.t.Valid {
		smoothed = current // первое значение — без сглаживания
	} else {
		smoothed = emaAlpha*current + (1-emaAlpha)*smoothed
	}
	s.t = Telemetry{SOC: soc, Voltage: voltage, Current: current, SmoothedCurrent: smoothed, Mos: mos, T1: t1, T2: t2, Valid: true, Updated: time.Now()}
	s.mu.Unlock()
}

func (s *BMSState) get() Telemetry {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.t
}

// ---------- Протокол JK BMS ----------

func buildJKCommand(cmdType byte) []byte {
	cmd := make([]byte, 20)
	cmd[0], cmd[1], cmd[2], cmd[3], cmd[4] = 0xAA, 0x55, 0x90, 0xEB, cmdType
	var sum byte
	for i := 0; i < 19; i++ {
		sum += cmd[i]
	}
	cmd[19] = sum
	return cmd
}

func processJKBuffer(buf []byte, state *BMSState) []byte {
	for {
		idx := bytes.Index(buf, headerResponseMagic)
		if idx == -1 {
			if len(buf) > 3 {
				return append([]byte(nil), buf[len(buf)-3:]...)
			}
			return buf
		}
		if idx > 0 {
			buf = buf[idx:]
		}
		if len(buf) < frameSize {
			return buf
		}

		frame := buf[:frameSize]
		var crc byte
		for _, b := range frame[:frameSize-1] {
			crc += b
		}
		if crc != frame[frameSize-1] {
			buf = buf[1:]
			continue
		}
		if frame[4] == 0x02 {
			parseTelemetry(frame, state)
		}
		buf = buf[frameSize:]
	}
}

func parseTelemetry(frame []byte, state *BMSState) {
	if len(frame) < frameSize {
		return
	}

	// SOC (%) — значение вне 0..100 считаем мусором и кадр отбрасываем
	soc := frame[173]
	if soc > 100 {
		return
	}

	// Ток разряда/заряда (int32 LE, 0.001 А), смещение 158
	rawCurrent := int32(binary.LittleEndian.Uint32(frame[158:162]))

	// Напряжение сборки (uint32 LE, 0.001 В), смещение 150 (118+32)
	voltage := float64(binary.LittleEndian.Uint32(frame[150:154])) * 0.001

	// Температуры (int16 LE, 0.1 °C): MOS 144:145, T1 162:163, T2 164:165
	mos := float64(int16(binary.LittleEndian.Uint16(frame[144:146]))) * 0.1
	t1 := float64(int16(binary.LittleEndian.Uint16(frame[162:164]))) * 0.1
	t2 := float64(int16(binary.LittleEndian.Uint16(frame[164:166]))) * 0.1
	state.update(uint16(soc), voltage, float64(rawCurrent)*0.001, mos, t1, t2)
}

// ---------- Telegram (асинхронно, с повторами) ----------

type Notifier struct {
	token, chatID string
	queue         chan string
}

func NewNotifier(token, chatID string) *Notifier {
	n := &Notifier{token: token, chatID: chatID, queue: make(chan string, 32)}
	go n.worker()
	return n
}

// Send не блокирует основной цикл, даже если интернета нет.
func (n *Notifier) Send(text string) {
	select {
	case n.queue <- text:
	default:
		writeLog("Очередь Telegram переполнена, сообщение отброшено")
	}
}

func (n *Notifier) worker() {
	for text := range n.queue {
		for attempt := 1; attempt <= 10; attempt++ {
			retry, err := n.post(text)
			if err == nil {
				break
			}
			if attempt == 1 || !retry || attempt == 10 {
				writeLog("Telegram: ошибка отправки (попытка %d): %v", attempt, err)
			}
			if !retry {
				break
			}
			time.Sleep(10 * time.Second)
		}
	}
}

func (n *Notifier) post(text string) (retry bool, err error) {
	body := url.Values{"chat_id": {n.chatID}, "text": {text}}.Encode()
	code, err := winhttpPost("api.telegram.org", "/bot"+n.token+"/sendMessage", body)
	if err != nil {
		// ошибки WinHTTP не содержат URL (а значит, и токен), но на всякий случай вырезаем его
		return true, errors.New(strings.ReplaceAll(err.Error(), n.token, "***"))
	}
	if code == 200 {
		return false, nil
	}
	// 4xx (кроме 429) — ошибка настроек (токен/chat_id), повторять бессмысленно
	permanent := code >= 400 && code < 500 && code != 429
	return !permanent, fmt.Errorf("HTTP %d", code)
}

// ---------- BLE: подключение и автопереподключение ----------

func ensureAdapter() error {
	if adapterReady {
		return nil
	}
	if err := adapter.Enable(); err != nil {
		return err
	}
	adapterReady = true
	return nil
}

// findDevice сканирует эфир не дольше timeout (без этого Scan может висеть бесконечно).
func findDevice(name string, timeout time.Duration) (bluetooth.ScanResult, bool) {
	var res bluetooth.ScanResult
	found := false

	timer := time.AfterFunc(timeout, func() { _ = adapter.StopScan() })
	defer timer.Stop()

	err := adapter.Scan(func(a *bluetooth.Adapter, r bluetooth.ScanResult) {
		if r.LocalName() == name {
			res = r
			found = true
			_ = a.StopScan()
		}
	})
	return res, found && err == nil
}

func connectWithTimeout(addr bluetooth.Address, timeout time.Duration) (bluetooth.Device, error) {
	type result struct {
		dev bluetooth.Device
		err error
	}
	ch := make(chan result, 1)
	go func() {
		d, err := adapter.Connect(addr, bluetooth.ConnectionParams{})
		ch <- result{d, err}
	}()

	select {
	case r := <-ch:
		return r.dev, r.err
	case <-time.After(timeout):
		// если подключение всё-таки завершится позже — аккуратно закрываем его
		go func() {
			if r := <-ch; r.err == nil {
				_ = r.dev.Disconnect()
			}
		}()
		return bluetooth.Device{}, errors.New("таймаут подключения")
	}
}

// waitUpdatedAfter: ждёт, пока телеметрия обновится строго позже момента after.
// after — снимок state.get().Updated, взятый ДО отправки команды; иначе «уснувший»
// стрим может дать свежий Updated от предыдущего кадра и мы ложно решим, что BMS отозвался.
// Возвращает true, если данные пришли, false — по таймауту.
func waitUpdatedAfter(state *BMSState, after time.Time, timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if u := state.get().Updated; !u.IsZero() && u.After(after) {
			return true
		}
		time.Sleep(50 * time.Millisecond)
	}
	return false
}

// writeWithTimeout: запись в характеристику с защитой от зависшего awaitAsyncOperation.
// WinRT-запись блокирует вызывающую горутины до завершения операции; если BLE-адаптер
// «лежит», без таймаута она может висеть неопределённо. По образцу connectWithTimeout.
func writeWithTimeout(c bluetooth.DeviceCharacteristic, p []byte, timeout time.Duration) error {
	type result struct {
		n   int
		err error
	}
	ch := make(chan result, 1)
	go func() {
		n, err := c.WriteWithoutResponse(p)
		ch <- result{n, err}
	}()

	select {
	case r := <-ch:
		if r.err != nil {
			return r.err
		}
		return nil
	case <-time.After(timeout):
		return errors.New("таймаут записи в BMS")
	}
}

// runSession: одно подключение к BMS. Возвращается при ошибке, потере данных или сигнале stop.
func runSession(cfg Config, state *BMSState, stop <-chan struct{}) (gotData bool, err error) {
	if err := ensureAdapter(); err != nil {
		return false, fmt.Errorf("ошибка включения BLE: %w", err)
	}

	writeLog("Поиск JK BMS с именем '%s'...", cfg.TargetBMSName)
	target, ok := findDevice(cfg.TargetBMSName, 20*time.Second)
	if !ok {
		return false, errors.New("JK BMS не найдена при сканировании")
	}

	writeLog("Подключение к %s (%s)...", cfg.TargetBMSName, target.Address.String())
	device, err := connectWithTimeout(target.Address, 15*time.Second)
	if err != nil {
		return false, fmt.Errorf("не удалось подключиться: %w", err)
	}
	defer device.Disconnect()

	services, err := device.DiscoverServices([]bluetooth.UUID{serviceUUID})
	if err != nil || len(services) == 0 {
		return false, errors.New("сервис FFE0 не найден")
	}
	chars, err := services[0].DiscoverCharacteristics([]bluetooth.UUID{charUUID})
	if err != nil || len(chars) == 0 {
		return false, errors.New("характеристика FFE1 не найдена")
	}
	bmsChar := chars[0]

	// Буфер приёма защищён мьютексом: callback'и BLE могут прийти из разных горутин
	var (
		rxMu sync.Mutex
		rx   = make([]byte, 0, 2048)
	)
	err = bmsChar.EnableNotifications(func(buf []byte) {
		rxMu.Lock()
		defer rxMu.Unlock()
		rx = append(rx, buf...)
		rx = processJKBuffer(rx, state)
		if len(rx) > maxRxBuffer {
			rx = rx[:0]
		}
	})
	if err != nil {
		return false, fmt.Errorf("ошибка подписки на уведомления: %w", err)
	}

	// Ровно как ESPHome: сначала 0x97 (один раз, при подключении),
	// потом повторяем 0x96 с интервалом 2с, пока BMS не начнёт стримить (0x02).
	kick := func() error {
		// 0x97 — один раз, «подготовка» (ESPHome: ESP_GATTC_REG_FOR_NOTIFY_EVT)
		if err := writeWithTimeout(bmsChar, buildJKCommand(0x97), kickTimeout); err != nil {
			return err
		}
		// 0x96 — повторяем, пока 0x02 не придёт (ESPHome: update())
		before := state.get().Updated
		deadline := time.Now().Add(30 * time.Second)
		for time.Now().Before(deadline) {
			if err := writeWithTimeout(bmsChar, buildJKCommand(0x96), kickTimeout); err != nil {
				return err
			}
			if waitUpdatedAfter(state, before, 2*time.Second) {
				return nil
			}
			time.Sleep(2 * time.Second)
		}
		return errors.New("BMS не отозвался на будоражение")
	}

	time.Sleep(500 * time.Millisecond)
	if err := kick(); err != nil {
		return false, fmt.Errorf("ошибка отправки команд: %w", err)
	}
	writeLog("Мониторинг JK BMS успешно запущен.")

	started := time.Now()
	lastKick := started
	ticker := time.NewTicker(time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-stop:
			return gotData, nil
		case <-ticker.C:
		}

		last := started
		if u := state.get().Updated; u.After(started) {
			last = u
			gotData = true
		}
		age := time.Since(last)

		if age > reconnectAfter {
			return gotData, fmt.Errorf("нет данных от BMS более %d с", int(reconnectAfter.Seconds()))
		}
		if age > kickAfter && time.Since(lastKick) > kickAfter {
			lastKick = time.Now()
			_ = kick() // если соединение мертво — это выяснится по таймауту выше
		}
	}
}

// bmsLoop держит соединение с BMS «вечно»: при любой проблеме переподключается с растущей паузой.
func bmsLoop(cfg Config, state *BMSState, stop <-chan struct{}, done chan<- struct{}) {
	defer close(done)
	delay := minRetry
	for {
		gotData, err := runSession(cfg, state, stop)

		select {
		case <-stop:
			return
		default:
		}

		if gotData {
			delay = minRetry
		}
		if err != nil {
			writeLog("BLE: %v. Повтор через %s", err, delay)
		}

		select {
		case <-stop:
			return
		case <-time.After(delay):
		}
		if !gotData && delay < maxRetry {
			delay *= 2
			if delay > maxRetry {
				delay = maxRetry
			}
		}
	}
}

// ---------- Вспомогательные функции ----------

// formatRemainingTime: расчёт оставшегося времени по СГЛАЖЕННОМУ току (EMA).
// Мгновенный ток скачет (балансировка, нагрузка, шум), из-за чего время «прыгает».
// EMA с alpha=0.1 сглаживает шум, но за ~10 с подстраивается под реальное изменение нагрузки.
func formatRemainingTime(soc uint16, smoothedCurrent, capacityAh float64) string {
	if smoothedCurrent >= dischargeLimit {
		return "н/д"
	}
	return fmtHM(capacityAh * (float64(soc) / 100.0) / math.Abs(smoothedCurrent))
}

// formatTimeToFull: время до полной зарядки по сглаженному току заряда (в том же стиле).
// Оценка линейная: ближе к 100% BMS снижает ток (фаза CV), поэтому реальное время будет чуть больше.
func formatTimeToFull(soc uint16, smoothedCurrent, capacityAh float64) string {
	if soc >= 100 {
		return "заряжена"
	}
	if smoothedCurrent <= -dischargeLimit { // заряжается слишком слабо (или не заряжается)
		return "н/д"
	}
	return fmtHM(capacityAh * (float64(100-soc) / 100.0) / smoothedCurrent)
}

func fmtHM(hours float64) string {
	h := int(hours)
	m := int((hours - float64(h)) * 60)
	if h > 0 {
		return fmt.Sprintf("%d ч %d мин", h, m)
	}
	return fmt.Sprintf("%d мин", m)
}

// trayTooltip собирает текст тултипа (лимит Windows — 127 символов, наш максимум ~100).
// Если тока практически нет (|I| < 0.1 А), строки «Ток» и «Осталось/До полной» не показываются.
func trayTooltip(t Telemetry, capacityAh float64) string {
	head := fmt.Sprintf("Заряд: %d%% · %.2f В", t.SOC, t.Voltage)
	temps := fmt.Sprintf("MOS %.1f · T1 %.1f · T2 %.1f °C", t.Mos, t.T1, t.T2)
	if math.Abs(t.Current) < -dischargeLimit {
		return head + "\n" + temps
	}

	var third string
	switch {
	case t.Current < 0:
		third = "Осталось: " + formatRemainingTime(t.SOC, t.SmoothedCurrent, capacityAh)
	case t.SOC >= 100:
		third = "Батарея заряжена"
	default:
		third = "До полной: " + formatTimeToFull(t.SOC, t.SmoothedCurrent, capacityAh)
	}
	power := math.Abs(t.Voltage * t.Current)
	return fmt.Sprintf("%s\nТок: %+.2f А · %.2f W\n%s\n%s", head, t.Current, power, third, temps)
}

func fmtDur(d time.Duration) string {
	s := int(d.Seconds())
	return fmt.Sprintf("%d мин %d сек", s/60, s%60)
}

func stamp() string { return time.Now().Format("02.01.2006 15:04:05") }

// hiddenCmd запускает консольную утилиту без мелькающего окна (важно при сборке с -H windowsgui).
func hiddenCmd(name string, args ...string) *exec.Cmd {
	c := exec.Command(name, args...)
	c.SysProcAttr = &syscall.SysProcAttr{HideWindow: true, CreationFlags: 0x08000000} // CREATE_NO_WINDOW
	return c
}

func shutdownCmd(timeoutSec int) *exec.Cmd {
	return hiddenCmd("shutdown", "/s", "/t", strconv.Itoa(timeoutSec))
}

func abortShutdownCmd() *exec.Cmd {
	return hiddenCmd("shutdown", "/a")
}

// ---------- WinHTTP (вместо net/http: экономит ~3.5 МБ в exe) ----------

var (
	winhttp                 = windows.NewLazySystemDLL("winhttp.dll")
	pWinHttpOpen            = winhttp.NewProc("WinHttpOpen")
	pWinHttpSetTimeouts     = winhttp.NewProc("WinHttpSetTimeouts")
	pWinHttpConnect         = winhttp.NewProc("WinHttpConnect")
	pWinHttpOpenRequest     = winhttp.NewProc("WinHttpOpenRequest")
	pWinHttpSendRequest     = winhttp.NewProc("WinHttpSendRequest")
	pWinHttpReceiveResponse = winhttp.NewProc("WinHttpReceiveResponse")
	pWinHttpQueryHeaders    = winhttp.NewProc("WinHttpQueryHeaders")
	pWinHttpCloseHandle     = winhttp.NewProc("WinHttpCloseHandle")
)

// winhttpPost отправляет POST (form-urlencoded) по HTTPS и возвращает HTTP-статус.
// Тексты ошибок не содержат path, поэтому токен в них не попадает.
// Указатели передаются прямо в Call(...) — так их держит живыми //go:uintptrescapes.
func winhttpPost(host, path, body string) (int, error) {
	const (
		accessAutoProxy = 4               // WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY (Win 8.1+)
		flagSecure      = 0x00800000      // WINHTTP_FLAG_SECURE
		queryStatus     = 19 | 0x20000000 // WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER
		timeoutMs       = 10000
	)
	fail := func(name string, e error) (int, error) {
		return 0, fmt.Errorf("%s: %v", name, e)
	}

	agent, _ := windows.UTF16PtrFromString("ups-jk-bms-monitor")
	hostP, _ := windows.UTF16PtrFromString(host)
	verbP, _ := windows.UTF16PtrFromString("POST")
	pathP, _ := windows.UTF16PtrFromString(path)
	hdrP, _ := windows.UTF16PtrFromString("Content-Type: application/x-www-form-urlencoded")
	b := []byte(body)
	if len(b) == 0 {
		return 0, errors.New("пустое тело запроса")
	}

	ses, _, e := pWinHttpOpen.Call(uintptr(unsafe.Pointer(agent)), accessAutoProxy, 0, 0, 0)
	if ses == 0 {
		return fail("WinHttpOpen", e)
	}
	defer pWinHttpCloseHandle.Call(ses)
	pWinHttpSetTimeouts.Call(ses, timeoutMs, timeoutMs, timeoutMs, timeoutMs)

	conn, _, e := pWinHttpConnect.Call(ses, uintptr(unsafe.Pointer(hostP)), 443, 0)
	if conn == 0 {
		return fail("WinHttpConnect", e)
	}
	defer pWinHttpCloseHandle.Call(conn)

	req, _, e := pWinHttpOpenRequest.Call(conn, uintptr(unsafe.Pointer(verbP)), uintptr(unsafe.Pointer(pathP)), 0, 0, 0, flagSecure)
	if req == 0 {
		return fail("WinHttpOpenRequest", e)
	}
	defer pWinHttpCloseHandle.Call(req)

	r, _, e := pWinHttpSendRequest.Call(req, uintptr(unsafe.Pointer(hdrP)), uintptr(^uint32(0)),
		uintptr(unsafe.Pointer(&b[0])), uintptr(len(b)), uintptr(len(b)), 0)
	if r == 0 {
		return fail("WinHttpSendRequest", e)
	}
	if r, _, e = pWinHttpReceiveResponse.Call(req, 0); r == 0 {
		return fail("WinHttpReceiveResponse", e)
	}

	var status uint32
	size := uint32(4)
	if r, _, e = pWinHttpQueryHeaders.Call(req, queryStatus, 0,
		uintptr(unsafe.Pointer(&status)), uintptr(unsafe.Pointer(&size)), 0); r == 0 {
		return fail("WinHttpQueryHeaders", e)
	}
	return int(status), nil
}

// ---------- Иконка в трее (чистый Win32, без сторонних библиотек) ----------

const (
	colGreen  uint32 = 0xFF1E8E3E // питание от сети
	colOrange uint32 = 0xFFE67700 // работа от батареи
	colRed    uint32 = 0xFFC62828 // низкий заряд
	colGray   uint32 = 0xFF666666 // нет связи с BMS
)

const (
	wmCommand   = 0x0111
	wmDestroy   = 0x0002
	wmClose     = 0x0010
	wmRButtonUp = 0x0205
	wmTray      = 0x8001 // WM_APP+1: события от иконки
	wmUpdate    = 0x8002 // WM_APP+2: перерисовать иконку
	idExit      = 1001

	nimAdd    = 0
	nimModify = 1
	nimDelete = 2

	nifMessage = 1
	nifIcon    = 2
	nifTip     = 4
)

var (
	user32  = windows.NewLazySystemDLL("user32.dll")
	shell32 = windows.NewLazySystemDLL("shell32.dll")
	gdi32   = windows.NewLazySystemDLL("gdi32.dll")

	pRegisterClassEx  = user32.NewProc("RegisterClassExW")
	pCreateWindowEx   = user32.NewProc("CreateWindowExW")
	pDefWindowProc    = user32.NewProc("DefWindowProcW")
	pDestroyWindow    = user32.NewProc("DestroyWindow")
	pGetMessage       = user32.NewProc("GetMessageW")
	pTranslateMessage = user32.NewProc("TranslateMessage")
	pDispatchMessage  = user32.NewProc("DispatchMessageW")
	pPostMessage      = user32.NewProc("PostMessageW")
	pPostQuitMessage  = user32.NewProc("PostQuitMessage")
	pRegisterWinMsg   = user32.NewProc("RegisterWindowMessageW")
	pCreateIconInd    = user32.NewProc("CreateIconIndirect")
	pDestroyIcon      = user32.NewProc("DestroyIcon")
	pGetSystemMetrics = user32.NewProc("GetSystemMetrics")
	pCreatePopupMenu  = user32.NewProc("CreatePopupMenu")
	pAppendMenu       = user32.NewProc("AppendMenuW")
	pTrackPopupMenu   = user32.NewProc("TrackPopupMenu")
	pDestroyMenu      = user32.NewProc("DestroyMenu")
	pGetCursorPos     = user32.NewProc("GetCursorPos")
	pSetForegroundWnd = user32.NewProc("SetForegroundWindow")
	pShellNotifyIcon  = shell32.NewProc("Shell_NotifyIconW")
	pCreateBitmap     = gdi32.NewProc("CreateBitmap")
	pDeleteObject     = gdi32.NewProc("DeleteObject")
)

type wndClassEx struct {
	cbSize, style                      uint32
	lpfnWndProc                        uintptr
	cbClsExtra, cbWndExtra             int32
	hInstance, hIcon, hCursor, hbrBack uintptr
	lpszMenuName, lpszClassName        *uint16
	hIconSm                            uintptr
}

type point struct{ x, y int32 }

type winMsg struct {
	hwnd    uintptr
	message uint32
	wParam  uintptr
	lParam  uintptr
	time    uint32
	pt      point
}

type iconInfo struct {
	fIcon              uint32
	xHotspot, yHotspot uint32
	hbmMask, hbmColor  uintptr
}

type notifyIconData struct {
	cbSize           uint32
	hWnd             uintptr
	uID, uFlags      uint32
	uCallbackMessage uint32
	hIcon            uintptr
	szTip            [128]uint16
	dwState, dwMask  uint32
	szInfo           [256]uint16
	uTimeout         uint32
	szInfoTitle      [64]uint16
	dwInfoFlags      uint32
	guidItem         windows.GUID
	hBalloonIcon     uintptr
}

// Шрифт 3×5, 15 бит на цифру, старший бит = левый верхний пиксель.
var digitFont = [10]uint16{
	0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9,
	0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF,
}

type trayState struct {
	soc   int
	color uint32
	tip   string
}

var (
	trayMu       sync.Mutex
	trayCur      = trayState{soc: -1, color: colGray, tip: "ups-jk-bms-monitor: ожидание данных..."}
	trayLastPost time.Time
	trayHwnd     atomic.Uintptr
	trayDone     = make(chan struct{})

	// используются только в потоке трея
	taskbarCreated uint32
	prevIcon       uintptr
	iconSOC        = -2
	iconColor      uint32
	onTrayExit     func()
)

// trayUpdate можно вызывать из любой горутины. Лишние вызовы отбрасываются:
// иконка перерисовывается при смене процента/цвета, тултип обновляется не чаще раза в 5 с.
func trayUpdate(soc int, color uint32, tip string) {
	trayMu.Lock()
	changed := soc != trayCur.soc || color != trayCur.color
	if !changed && time.Since(trayLastPost) < 5*time.Second {
		trayMu.Unlock()
		return
	}
	trayCur = trayState{soc, color, tip}
	trayLastPost = time.Now()
	trayMu.Unlock()
	if h := trayHwnd.Load(); h != 0 {
		pPostMessage.Call(h, wmUpdate, 0, 0)
	}
}

// trayStop убирает иконку и ждёт завершения потока трея (чтобы не осталась «призрачная» иконка).
func trayStop() {
	if h := trayHwnd.Load(); h != 0 {
		pPostMessage.Call(h, wmClose, 0, 0)
		select {
		case <-trayDone:
		case <-time.After(time.Second):
		}
	}
}

func makeIcon(soc int, bg uint32) uintptr {
	sz := 16
	if v, _, _ := pGetSystemMetrics.Call(49); v > 0 { // SM_CXSMICON (учитывает DPI)
		sz = int(v)
	}
	px := make([]uint32, sz*sz)
	for i := range px {
		px[i] = bg
	}
	text := "--"
	if soc >= 0 {
		text = strconv.Itoa(soc)
	}
	n := len(text)
	k := sz / (4*n - 1) // масштаб «пикселя» шрифта
	if m := (sz - 2) / 5; k > m {
		k = m
	}
	if k < 1 {
		k = 1
	}
	w, h := (4*n-1)*k, 5*k
	x0, y0 := (sz-w)/2, (sz-h)/2
	for i := 0; i < n; i++ {
		if text[i] < '0' || text[i] > '9' { // «-»: горизонтальная черта
			for dx := 0; dx < 3*k; dx++ {
				for dy := 0; dy < k; dy++ {
					px[(y0+2*k+dy)*sz+x0+i*4*k+dx] = 0xFFFFFFFF
				}
			}
			continue
		}
		g := digitFont[text[i]-'0']
		for r := 0; r < 5; r++ {
			for c := 0; c < 3; c++ {
				if g&(1<<uint(14-(r*3+c))) == 0 {
					continue
				}
				for dy := 0; dy < k; dy++ {
					for dx := 0; dx < k; dx++ {
						px[(y0+r*k+dy)*sz+x0+(i*4+c)*k+dx] = 0xFFFFFFFF
					}
				}
			}
		}
	}
	color, _, _ := pCreateBitmap.Call(uintptr(sz), uintptr(sz), 1, 32, uintptr(unsafe.Pointer(&px[0])))
	mask, _, _ := pCreateBitmap.Call(uintptr(sz), uintptr(sz), 1, 1, 0)
	ii := iconInfo{fIcon: 1, hbmMask: mask, hbmColor: color}
	hIcon, _, _ := pCreateIconInd.Call(uintptr(unsafe.Pointer(&ii)))
	pDeleteObject.Call(color)
	pDeleteObject.Call(mask)
	return hIcon
}

func applyIcon(hwnd uintptr, op uintptr) {
	trayMu.Lock()
	s := trayCur
	trayMu.Unlock()

	// Иконку пересоздаём только если изменились цифры/цвет
	if s.soc != iconSOC || s.color != iconColor || prevIcon == 0 {
		old := prevIcon
		prevIcon = makeIcon(s.soc, s.color)
		iconSOC, iconColor = s.soc, s.color
		defer func() {
			if old != 0 {
				pDestroyIcon.Call(old)
			}
		}()
	}

	nid := notifyIconData{hWnd: hwnd, uID: 1, uFlags: nifMessage | nifIcon | nifTip,
		uCallbackMessage: wmTray, hIcon: prevIcon}
	nid.cbSize = uint32(unsafe.Sizeof(nid))
	if u, err := windows.UTF16FromString(s.tip); err == nil {
		if len(u) > 127 {
			u = u[:127]
		}
		copy(nid.szTip[:], u)
	}
	pShellNotifyIcon.Call(op, uintptr(unsafe.Pointer(&nid)))
}

func showMenu(hwnd uintptr) {
	menu, _, _ := pCreatePopupMenu.Call()
	text, _ := windows.UTF16PtrFromString("Выход")
	pAppendMenu.Call(menu, 0, idExit, uintptr(unsafe.Pointer(text)))
	var p point
	pGetCursorPos.Call(uintptr(unsafe.Pointer(&p)))
	pSetForegroundWnd.Call(hwnd) // иначе меню не закроется кликом мимо
	pTrackPopupMenu.Call(menu, 0x20|0x2, uintptr(p.x), uintptr(p.y), 0, hwnd, 0)
	pPostMessage.Call(hwnd, 0, 0, 0) // WM_NULL
	pDestroyMenu.Call(menu)
}

func wndProc(hwnd, msg, wp, lp uintptr) uintptr {
	if taskbarCreated != 0 && uint32(msg) == taskbarCreated { // Explorer перезапустился
		applyIcon(hwnd, nimAdd)
		return 0
	}
	switch msg {
	case wmTray:
		if lp == wmRButtonUp {
			showMenu(hwnd)
		}
		return 0
	case wmUpdate:
		applyIcon(hwnd, nimModify)
		return 0
	case wmCommand:
		if wp&0xFFFF == idExit {
			pDestroyWindow.Call(hwnd)
			if onTrayExit != nil {
				onTrayExit()
			}
		}
		return 0
	case wmDestroy:
		nid := notifyIconData{hWnd: hwnd, uID: 1}
		nid.cbSize = uint32(unsafe.Sizeof(nid))
		pShellNotifyIcon.Call(nimDelete, uintptr(unsafe.Pointer(&nid)))
		if prevIcon != 0 {
			pDestroyIcon.Call(prevIcon)
			prevIcon = 0
		}
		pPostQuitMessage.Call(0)
		return 0
	}
	r, _, _ := pDefWindowProc.Call(hwnd, msg, wp, lp)
	return r
}

// runTray запускается в отдельной горутине: go runTray(func(){...}).
// Поток блокируется в GetMessage — в простое CPU не потребляет.
func runTray(onExit func()) {
	runtime.LockOSThread()
	defer close(trayDone)
	onTrayExit = onExit

	name, _ := windows.UTF16PtrFromString("UPSJKBMSTray")
	tb, _ := windows.UTF16PtrFromString("TaskbarCreated")
	r, _, _ := pRegisterWinMsg.Call(uintptr(unsafe.Pointer(tb)))
	taskbarCreated = uint32(r)

	var hInst windows.Handle
	_ = windows.GetModuleHandleEx(0, nil, &hInst)
	wc := wndClassEx{lpfnWndProc: syscall.NewCallback(wndProc), hInstance: uintptr(hInst), lpszClassName: name}
	wc.cbSize = uint32(unsafe.Sizeof(wc))
	pRegisterClassEx.Call(uintptr(unsafe.Pointer(&wc)))

	// Обычное скрытое окно (не message-only), иначе не придёт TaskbarCreated
	hwnd, _, _ := pCreateWindowEx.Call(0, uintptr(unsafe.Pointer(name)), uintptr(unsafe.Pointer(name)),
		0, 0, 0, 0, 0, 0, 0, uintptr(hInst), 0)
	if hwnd == 0 {
		return
	}
	applyIcon(hwnd, nimAdd)
	trayHwnd.Store(hwnd)

	var m winMsg
	for {
		r, _, _ := pGetMessage.Call(uintptr(unsafe.Pointer(&m)), 0, 0, 0)
		if int32(r) <= 0 {
			break
		}
		pTranslateMessage.Call(uintptr(unsafe.Pointer(&m)))
		pDispatchMessage.Call(uintptr(unsafe.Pointer(&m)))
	}
}

// ---------- main ----------

func main() {
	dir := baseDir()
	env := loadEnv(filepath.Join(dir, ".env"))

	logName := env["LOG_FILE"]
	if logName == "" {
		logName = "ups_log.txt"
	}
	if !filepath.IsAbs(logName) {
		logName = filepath.Join(dir, logName)
	}
	logPath = logName

	token, chatID := env["TELEGRAM_TOKEN"], env["TELEGRAM_CHAT_ID"]
	if token == "" || chatID == "" {
		fatal("Ошибка: TELEGRAM_TOKEN или TELEGRAM_CHAT_ID не заданы в .env!")
	}
	if env["TARGET_BMS_NAME"] == "" {
		fatal("Ошибка: TARGET_BMS_NAME не задан в .env!")
	}

	cfg := Config{
		TargetBMSName:       env["TARGET_BMS_NAME"],
		BatteryCapacityAh:   envNum(env, "BATTERY_CAPACITY_AH", 15, 0.1),
		DelayNotify:         envNum(env, "DELAY_NOTIFY", 60, 0),
		ShutdownThreshold:   uint16(envNum(env, "SHUTDOWN_THRESHOLD", 10, 1)),
		LowBatteryThreshold: uint16(envNum(env, "LOW_BATTERY_THRESHOLD", 20, 1)),
		ShutdownTimeout:     int(envNum(env, "SHUTDOWN_TIMEOUT", 30, 1)),
		StaleShutdown:       int(envNum(env, "STALE_SHUTDOWN", 0, 0)),
		TempThreshold:       envNum(env, "TEMP_THRESHOLD", 69, 0),
		TempHysteresis:      envNum(env, "TEMP_HYSTERESIS", 5, 0),
	}
	if cfg.ShutdownThreshold > 100 {
		writeLog("Предупреждение: SHUTDOWN_THRESHOLD > 100, используется 10")
		cfg.ShutdownThreshold = 10
	}

	notifier := NewNotifier(token, chatID)
	var state BMSState

	stop := make(chan struct{})
	done := make(chan struct{})
	go bmsLoop(cfg, &state, stop, done)

	quit := make(chan struct{}, 1) // выход из меню трея
	go runTray(func() {
		select {
		case quit <- struct{}{}:
		default:
		}
	})

	var (
		onBattery          bool // подтверждённое состояние (с антидребезгом)
		pending            int  // сколько тиков подряд наблюдается противоположное состояние
		notified           bool
		lowBatteryNotified bool // предупреждение о низком заряде уже отправлено
		shutdownStarted    bool
		stale              bool
		lossTime           time.Time
		chargeAtLoss       uint16
		smoothedAtLoss     float64 // снимок сглаженного тока в момент отключения (для согласованности лог/Telegram)
		nextShutdownTry    time.Time
		tempAlerted        bool // предупреждение о температуре уже отправлено (сработает повторно после отката ниже порога)
	)

	doShutdown := func(reason string) {
		if shutdownStarted || time.Now().Before(nextShutdownTry) {
			return
		}
		writeLog("%s Инициировано выключение ПК.", reason)
		if err := shutdownCmd(cfg.ShutdownTimeout).Run(); err != nil {
			writeLog("Не удалось выполнить shutdown: %v", err)
			nextShutdownTry = time.Now().Add(10 * time.Second)
			return
		}
		shutdownStarted = true
		notifier.Send(fmt.Sprintf("%s — Выключение ПК!\n%s", stamp(), reason))
	}

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	ticker := time.NewTicker(time.Second)
	defer ticker.Stop()

	for {
		exiting := false
		select {
		case <-sigChan:
			writeLog("Получен сигнал завершения. Остановка службы...")
			exiting = true
		case <-quit:
			writeLog("Выход из меню трея. Остановка службы...")
			exiting = true
		case <-ticker.C:
		}
		if exiting {
			close(stop)
			select {
			case <-done:
			case <-time.After(5 * time.Second):
			}
			trayStop()
			return
		}

		t := state.get()
		if !t.Valid {
			continue
		}

		// Устаревшие данные не используем для принятия решений
		age := time.Since(t.Updated)
		if age > staleAfter {
			if !stale {
				stale = true
				writeLog("Нет данных от BMS более %d с, идёт переподключение...", int(staleAfter.Seconds()))
			}
			trayUpdate(-1, colGray, "Нет связи с BMS")
			// Страховка: на батарее и связи с BMS нет — не ждём, пока батарея сядет «вслепую»
			if onBattery && cfg.StaleShutdown > 0 && age > time.Duration(cfg.StaleShutdown)*time.Second {
				doShutdown(fmt.Sprintf("Связь с BMS потеряна на %d с при работе от батареи (последний заряд: %d%%).",
					int(age.Seconds()), t.SOC))
			}
			continue
		}
		if stale {
			stale = false
			writeLog("Связь с BMS восстановлена.")
		}

		// Антидребезг: смена состояния питания должна продержаться debounceTicks секунд
		battery := t.Current < dischargeLimit
		if battery == onBattery {
			pending = 0
		} else {
			pending++
		}

		if pending >= debounceTicks {
			pending = 0
			if battery {
				// 1. Отключение питания
				onBattery = true
				notified = false
				lossTime = time.Now()
				chargeAtLoss = t.SOC
				smoothedAtLoss = t.SmoothedCurrent // снимок для согласованности лог/Telegram
				writeLog("Отключение электричества! Заряд: %d%%, осталось: %s.",
					chargeAtLoss, formatRemainingTime(chargeAtLoss, smoothedAtLoss, cfg.BatteryCapacityAh))
			} else {
				// 3. Восстановление питания
				duration := time.Since(lossTime)
				writeLog("Электричество восстановлено. Заряд: %d%%, прошло: %s.", t.SOC, fmtDur(duration))

				if shutdownStarted {
					if err := abortShutdownCmd().Run(); err != nil {
						writeLog("Не удалось отменить выключение: %v", err)
					} else {
						writeLog("Выключение ПК отменено.")
						notifier.Send(fmt.Sprintf("%s — Питание вернулось, выключение ПК отменено.", stamp()))
					}
					shutdownStarted = false
				}
				if notified {
					notifier.Send(fmt.Sprintf("%s — Электричество восстановлено.\nЗаряд: %d%%, прошло: %s.",
						stamp(), t.SOC, fmtDur(duration)))
				}
				onBattery = false
				notified = false
				lowBatteryNotified = false
			}
		}

		// Иконка в трее: цвет и процент заряда, в тултипе — подробности
		trayCol := colGreen
		if onBattery {
			trayCol = colOrange
			if t.SOC <= cfg.LowBatteryThreshold {
				trayCol = colRed
			}
		}
		trayUpdate(int(t.SOC), trayCol, trayTooltip(t, cfg.BatteryCapacityAh))

		if !onBattery {
			continue
		}

		// 2. Уведомление в Telegram с задержкой (чтобы не спамить из-за коротких пропаданий).
		// Использует СНИМКИ (chargeAtLoss, smoothedAtLoss) из момента отключения,
		// чтобы совпадать с логом (иначе EMA подстроится за DELAY_NOTIFY секунд).
		if !notified && time.Since(lossTime).Seconds() >= cfg.DelayNotify {
			notifier.Send(fmt.Sprintf("%s — Отключение электричества!\nЗаряд: %d%%, осталось: %s.",
				lossTime.Format("02.01.2006 15:04:05"), chargeAtLoss,
				formatRemainingTime(chargeAtLoss, smoothedAtLoss, cfg.BatteryCapacityAh)))
			notified = true
		}

		// 3. Предупреждение о низком заряде (разовое, до аварийного выключения)
		if !lowBatteryNotified && t.SOC <= cfg.LowBatteryThreshold {
			notifier.Send(fmt.Sprintf("%s — Низкий заряд: %d%%. Осталось ~%s.",
				stamp(), t.SOC, formatRemainingTime(t.SOC, t.SmoothedCurrent, cfg.BatteryCapacityAh)))
			lowBatteryNotified = true
		}

		// 4. Выключение ПК при критическом заряде
		if t.SOC <= cfg.ShutdownThreshold {
			doShutdown(fmt.Sprintf("Критический уровень заряда (%d%%), на батарее: %s.",
				t.SOC, fmtDur(time.Since(lossTime))))
		}

		// 5. Предупреждение о высокой температуре (MOS / T1 / T2)
		// Разовый алерт при превышении порога; повторно — только после отката
		// всех датчиков ниже порога на TEMP_HYSTERESIS.
		if !tempAlerted {
			if t.Mos >= cfg.TempThreshold {
				writeLog("Высокая температура MOS: %g°C.", t.Mos)
				notifier.Send(fmt.Sprintf("%s — Высокая температура MOS: %g°C.", stamp(), t.Mos))
				tempAlerted = true
			}
			if t.T1 >= cfg.TempThreshold {
				writeLog("Высокая температура T1: %g°C.", t.T1)
				notifier.Send(fmt.Sprintf("%s — Высокая температура T1: %g°C.", stamp(), t.T1))
				tempAlerted = true
			}
			if t.T2 >= cfg.TempThreshold {
				writeLog("Высокая температура T2: %g°C.", t.T2)
				notifier.Send(fmt.Sprintf("%s — Высокая температура T2: %g°C.", stamp(), t.T2))
				tempAlerted = true
			}
		} else if t.Mos < cfg.TempThreshold-cfg.TempHysteresis &&
			t.T1 < cfg.TempThreshold-cfg.TempHysteresis && t.T2 < cfg.TempThreshold-cfg.TempHysteresis {
			tempAlerted = false
		}
	}
}
