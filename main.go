package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"net/http"
	"net/url"
	"os"
	"os/exec"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"tinygo.org/x/bluetooth"
)

var (
	serviceUUID, _      = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	charUUID, _         = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
	headerResponseMagic = []byte{0x55, 0xAA, 0xEB, 0x90}
	adapter             = bluetooth.DefaultAdapter
)

type Config struct {
	TargetBMSName     string
	BatteryCapacityAh float64
	TelegramToken     string
	TelegramChatID    string
	DelayNotify       float64
	ShutdownThreshold uint16
	ShutdownTimeout   int
	LogFile           string
}

type BMSState struct {
	mu           sync.Mutex
	SOC          uint16
	Current      float64
	HasTelemetry bool
	LastUpdated  time.Time
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
		line := strings.TrimSpace(scanner.Text())
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		parts := strings.SplitN(line, "=", 2)
		if len(parts) == 2 {
			env[strings.TrimSpace(parts[0])] = strings.TrimSpace(parts[1])
		}
	}
	if err := scanner.Err(); err != nil {
		log.Printf("Ошибка чтения файла окружения: %v", err)
	}
	return env
}

func writeLog(logFile, message string) {
	timestamp := time.Now().Format("02.01.2006 15:04:05")
	entry := fmt.Sprintf("%s - %s\n", timestamp, message)
	fmt.Print(entry)

	f, err := os.OpenFile(logFile, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		log.Printf("Ошибка записи в лог: %v", err)
		return
	}
	defer f.Close()
	f.WriteString(entry)
}

func sendTelegram(token, chatID, text string) bool {
	apiURL := fmt.Sprintf("https://api.telegram.org/bot%s/sendMessage", token)
	data := url.Values{}
	data.Set("chat_id", chatID)
	data.Set("text", text)

	client := http.Client{Timeout: 5 * time.Second}
	resp, err := client.PostForm(apiURL, data)
	if err != nil {
		return false
	}
	defer resp.Body.Close()

	return resp.StatusCode == http.StatusOK
}

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
	const frameSize = 300

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
		for _, b := range frame[:299] {
			crc += b
		}

		if crc == frame[299] {
			if frame[4] == 0x02 {
				parseTelemetry(frame, state)
			}
		} else {
			buf = buf[1:]
			continue
		}

		buf = buf[frameSize:]
	}
}

func parseTelemetry(frame []byte, state *BMSState) {
	if len(frame) < 300 {
		return
	}

	state.mu.Lock()
	defer state.mu.Unlock()

	// Ток разряда/заряда (int32 LE, 0.001 A)
	rawCurrent := int32(binary.LittleEndian.Uint32(frame[126+32 : 130+32]))
	state.Current = float64(rawCurrent) * 0.001

	// SOC (%)
	if len(frame) > 173 {
		state.SOC = uint16(frame[173])
	}

	state.HasTelemetry = true
	state.LastUpdated = time.Now()
}

// Форматирование оставшегося времени в часы и минуты
func formatRemainingTime(soc uint16, current float64, capacityAh float64) string {
	if current >= -0.1 {
		return "0 мин"
	}
	dischargeCurrent := math.Abs(current)
	remainingAh := capacityAh * (float64(soc) / 100.0)
	totalHours := remainingAh / dischargeCurrent

	h := int(totalHours)
	m := int((totalHours - float64(h)) * 60)

	if h > 0 {
		return fmt.Sprintf("%d ч %d мин", h, m)
	}
	return fmt.Sprintf("%d мин", m)
}

func main() {
	env := loadEnv(".env")

	logFile := env["LOG_FILE"]
	if logFile == "" {
		logFile = "ups_log.txt"
	}

	token := env["TELEGRAM_TOKEN"]
	chatID := env["TELEGRAM_CHAT_ID"]
	if token == "" || chatID == "" {
		writeLog(logFile, "Ошибка: TELEGRAM_TOKEN или TELEGRAM_CHAT_ID не заданы в .env!")
		os.Exit(1)
	}

	targetBMSName := env["TARGET_BMS_NAME"]
	if targetBMSName == "" {
		writeLog(logFile, "Ошибка: TARGET_BMS_NAME не задан в .env!")
		os.Exit(1)
	}

	capacityAh, _ := strconv.ParseFloat(env["BATTERY_CAPACITY_AH"], 64)
	if capacityAh <= 0 {
		capacityAh = 15.0
	}

	delayNotifySec, _ := strconv.ParseFloat(env["DELAY_NOTIFY"], 64)
	if delayNotifySec < 0 {
		delayNotifySec = 60
	}

	shutdownThreshold, _ := strconv.Atoi(env["SHUTDOWN_THRESHOLD"])
	if shutdownThreshold <= 0 {
		shutdownThreshold = 10
	}

	shutdownTimeout, _ := strconv.Atoi(env["SHUTDOWN_TIMEOUT"])
	if shutdownTimeout <= 0 {
		shutdownTimeout = 30
	}

	cfg := Config{
		TargetBMSName:     targetBMSName,
		BatteryCapacityAh: capacityAh,
		TelegramToken:     token,
		TelegramChatID:    chatID,
		DelayNotify:       delayNotifySec,
		ShutdownThreshold: uint16(shutdownThreshold),
		ShutdownTimeout:   shutdownTimeout,
		LogFile:           logFile,
	}

	writeLog(cfg.LogFile, "Инициализация Bluetooth адаптера...")
	if err := adapter.Enable(); err != nil {
		writeLog(cfg.LogFile, fmt.Sprintf("Ошибка включения BLE: %v", err))
		os.Exit(1)
	}

	writeLog(cfg.LogFile, fmt.Sprintf("Поиск JK BMS с именем '%s'...", cfg.TargetBMSName))
	var targetDevice bluetooth.ScanResult
	found := false

	err := adapter.Scan(func(a *bluetooth.Adapter, result bluetooth.ScanResult) {
		if result.LocalName() == cfg.TargetBMSName {
			targetDevice = result
			found = true
			a.StopScan()
		}
	})

	if err != nil || !found {
		writeLog(cfg.LogFile, "JK BMS не найдена при сканировании!")
		os.Exit(1)
	}

	writeLog(cfg.LogFile, fmt.Sprintf("Подключение к %s (%s)...", cfg.TargetBMSName, targetDevice.Address.String()))
	device, err := adapter.Connect(targetDevice.Address, bluetooth.ConnectionParams{})
	if err != nil {
		writeLog(cfg.LogFile, fmt.Sprintf("Не удалось подключиться: %v", err))
		os.Exit(1)
	}
	defer device.Disconnect()

	services, err := device.DiscoverServices([]bluetooth.UUID{serviceUUID})
	if err != nil || len(services) == 0 {
		writeLog(cfg.LogFile, "Сервис FFE0 не найден!")
		os.Exit(1)
	}

	chars, err := services[0].DiscoverCharacteristics([]bluetooth.UUID{charUUID})
	if err != nil || len(chars) == 0 {
		writeLog(cfg.LogFile, "Характеристика FFE1 не найдена!")
		os.Exit(1)
	}

	bmsChar := chars[0]
	rxBuffer := make([]byte, 0, 2048)
	var state BMSState

	err = bmsChar.EnableNotifications(func(buf []byte) {
		rxBuffer = append(rxBuffer, buf...)
		rxBuffer = processJKBuffer(rxBuffer, &state)
	})
	if err != nil {
		writeLog(cfg.LogFile, fmt.Sprintf("Ошибка подписки на уведомления: %v", err))
		os.Exit(1)
	}

	time.Sleep(500 * time.Millisecond)

	// Инициализация последовательности подписки для активации стрима
	_, _ = bmsChar.WriteWithoutResponse(buildJKCommand(0x97))
	time.Sleep(700 * time.Millisecond)
	_, _ = bmsChar.WriteWithoutResponse(buildJKCommand(0x96))

	writeLog(cfg.LogFile, "Мониторинг JK BMS успешно запущен.")

	var (
		wasOnBattery     bool
		telegramNotified bool
		powerLostTime    time.Time
		chargeAtLoss     uint16
	)

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-sigChan:
			writeLog(cfg.LogFile, "Получен сигнал завершения. Остановка службы...")
			return

		case <-ticker.C:
			state.mu.Lock()
			hasData := state.HasTelemetry
			charge := state.SOC
			current := state.Current
			lastUpdated := state.LastUpdated
			state.mu.Unlock()

			if !hasData {
				continue
			}

			if time.Since(lastUpdated) > 15*time.Second {
				writeLog(cfg.LogFile, "Предупреждение: Данные от BMS не поступают более 15 секунд!")
			}

			// Проверка питания: разряд фиксируем только если ток меньше -0.1 А
			plugged := current >= -0.1
			remainingTimeStr := formatRemainingTime(charge, current, cfg.BatteryCapacityAh)

			// 1. Фиксация отключения питания
			if !plugged && !wasOnBattery {
				wasOnBattery = true
				powerLostTime = time.Now()
				chargeAtLoss = charge
				msg := fmt.Sprintf("Отключение электричества! Заряд: %d%%, осталось: %s.",
					chargeAtLoss, remainingTimeStr)
				writeLog(cfg.LogFile, msg)
			}

			// 2. Уведомление в Telegram с задержкой
			if wasOnBattery && !telegramNotified && !plugged {
				elapsed := time.Since(powerLostTime).Seconds()
				if elapsed >= cfg.DelayNotify {
					msg := fmt.Sprintf("%s — Отключение электричества!\nЗаряд: %d%%, осталось ~%s.",
						powerLostTime.Format("02.01.2006 15:04:05"), chargeAtLoss, remainingTimeStr)
					if sendTelegram(cfg.TelegramToken, cfg.TelegramChatID, msg) {
						telegramNotified = true
					}
				}
			}

			// 3. Восстановление питания
			if plugged && wasOnBattery {
				duration := int(time.Since(powerLostTime).Seconds())
				msg := fmt.Sprintf("Электричество восстановлено. Заряд: %d%%, прошло: %d мин %d сек.",
					charge, duration/60, duration%60)
				writeLog(cfg.LogFile, msg)

				if float64(duration) >= cfg.DelayNotify {
					sendTelegram(cfg.TelegramToken, cfg.TelegramChatID,
						fmt.Sprintf("%s — Электричество восстановлено.\nЗаряд: %d%%, прошло: %d мин %d сек.",
							time.Now().Format("02.01.2006 15:04:05"), charge, duration/60, duration%60))
				}

				wasOnBattery = false
				telegramNotified = false
			}

			// 4. Выключение ПК при критическом заряде
			if wasOnBattery && charge > 0 && charge <= cfg.ShutdownThreshold {
				duration := int(time.Since(powerLostTime).Seconds())
				msg := fmt.Sprintf("Критический уровень заряда (%d%%)! Инициировано выключение ПК.", charge)
				writeLog(cfg.LogFile, msg)

				sendTelegram(cfg.TelegramToken, cfg.TelegramChatID,
					fmt.Sprintf("%s — Выключение ПК!\nЗаряд: %d%%, прошло: %d мин %d сек.",
						time.Now().Format("02.01.2006 15:04:05"), charge, duration/60, duration%60))

				cmd := exec.Command("shutdown", "/s", "/t", strconv.Itoa(cfg.ShutdownTimeout))
				_ = cmd.Run()
				os.Exit(0)
			}
		}
	}
}
