package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"net/http"
	"net/url"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

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
	Current         float64 // мгновенный ток (А)
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

func (s *BMSState) update(soc uint16, current, mos, t1, t2 float64) {
	s.mu.Lock()
	// EMA: новое сглаженное = alpha * мгновенное + (1 - alpha) * предыдущее сглаженное
	smoothed := s.t.SmoothedCurrent
	if !s.t.Valid {
		smoothed = current // первое значение — без сглаживания
	} else {
		smoothed = emaAlpha*current + (1-emaAlpha)*smoothed
	}
	s.t = Telemetry{SOC: soc, Current: current, SmoothedCurrent: smoothed, Mos: mos, T1: t1, T2: t2, Valid: true, Updated: time.Now()}
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

	// Температуры (int16 LE, 0.1 °C): MOS 144:145, T1 162:163, T2 164:165
	mos := float64(int16(binary.LittleEndian.Uint16(frame[144:146]))) * 0.1
	t1 := float64(int16(binary.LittleEndian.Uint16(frame[162:164]))) * 0.1
	t2 := float64(int16(binary.LittleEndian.Uint16(frame[164:166]))) * 0.1
	state.update(uint16(soc), float64(rawCurrent)*0.001, mos, t1, t2)
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
	data := url.Values{"chat_id": {n.chatID}, "text": {text}}
	client := http.Client{Timeout: 10 * time.Second}
	resp, err := client.PostForm("https://api.telegram.org/bot"+n.token+"/sendMessage", data)
	if err != nil {
		// текст ошибки содержит URL с токеном — вырезаем его
		return true, errors.New(strings.ReplaceAll(err.Error(), n.token, "***"))
	}
	defer resp.Body.Close()
	if resp.StatusCode == http.StatusOK {
		return false, nil
	}
	// 4xx (кроме 429) — ошибка настроек (токен/chat_id), повторять бессмысленно
	permanent := resp.StatusCode >= 400 && resp.StatusCode < 500 && resp.StatusCode != http.StatusTooManyRequests
	return !permanent, fmt.Errorf("HTTP %d", resp.StatusCode)
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
	hours := capacityAh * (float64(soc) / 100.0) / math.Abs(smoothedCurrent)
	h := int(hours)
	m := int((hours - float64(h)) * 60)
	if h > 0 {
		return fmt.Sprintf("%d ч %d мин", h, m)
	}
	return fmt.Sprintf("%d мин", m)
}

func fmtDur(d time.Duration) string {
	s := int(d.Seconds())
	return fmt.Sprintf("%d мин %d сек", s/60, s%60)
}

func stamp() string { return time.Now().Format("02.01.2006 15:04:05") }

func shutdownCmd(timeoutSec int) *exec.Cmd {
	if runtime.GOOS == "windows" {
		return exec.Command("shutdown", "/s", "/t", strconv.Itoa(timeoutSec))
	}
	return exec.Command("shutdown", "-h", fmt.Sprintf("+%d", (timeoutSec+59)/60))
}

func abortShutdownCmd() *exec.Cmd {
	if runtime.GOOS == "windows" {
		return exec.Command("shutdown", "/a")
	}
	return exec.Command("shutdown", "-c")
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
		select {
		case <-sigChan:
			writeLog("Получен сигнал завершения. Остановка службы...")
			close(stop)
			select {
			case <-done:
			case <-time.After(5 * time.Second):
			}
			return
		case <-ticker.C:
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
