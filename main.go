package main

import (
	"encoding/hex"
	"fmt"
	"log"
	"strings"
	"time"

	"tinygo.org/x/bluetooth"
)

// Целевое имя или MAC вашей BMS
const TargetBMSName = "60126310241"
const TargetBMSAddress = "C8:47:80:53:5E:04"

var (
	serviceUUID, _ = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	charUUID, _    = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
)

// Запрос кадра телеметрии JK02/JK04
var requestFrame = []byte{
	0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x11,
}

var adapter = bluetooth.DefaultAdapter

func main() {
	fmt.Println("Инициализация Bluetooth адаптера...")
	if err := adapter.Enable(); err != nil {
		log.Fatalf("Ошибка включения BLE: %v", err)
	}

	fmt.Println("Поиск JK BMS...")

	var targetDevice bluetooth.ScanResult
	found := false

	err := adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		name := result.LocalName()
		addr := result.Address.String()

		// Проверяем совпадение по имени или MAC-адресу
		if name == TargetBMSName || strings.EqualFold(addr, TargetBMSAddress) {
			fmt.Printf("\n[УСПЕХ] Найдена JK BMS! Name: %s | Address: %s\n", name, addr)
			targetDevice = result
			found = true
			adapter.StopScan()
		}
	})

	if err != nil {
		log.Fatalf("Ошибка сканирования: %v", err)
	}

	if !found {
		log.Fatal("Устройство не найдено в зоне видимости BLE.")
	}

	// Подключение к BMS
	fmt.Printf("Подключение к %s...\n", targetDevice.Address.String())
	device, err := adapter.Connect(targetDevice.Address, bluetooth.ConnectionParams{})
	if err != nil {
		log.Fatalf("Не удалось подключиться: %v", err)
	}
	defer device.Disconnect()

	fmt.Println("Подключено! Поиск сервиса FFE0...")

	services, err := device.DiscoverServices([]bluetooth.UUID{serviceUUID})
	if err != nil || len(services) == 0 {
		log.Fatalf("Сервис FFE0 не найден: %v", err)
	}

	chars, err := services[0].DiscoverCharacteristics([]bluetooth.UUID{charUUID})
	if err != nil || len(chars) == 0 {
		log.Fatalf("Характеристика FFE1 не найдена: %v", err)
	}

	bmsChar := chars[0]

	// Буфер для сборки кадра
	rxBuffer := make([]byte, 0, 500)

	err = bmsChar.EnableNotifications(func(buf []byte) {
		rxBuffer = append(rxBuffer, buf...)
		fmt.Printf("-> Чанк BLE (%d байт), итого в буфере: %d байт\n", len(buf), len(rxBuffer))

		// Ищем заголовок кадра 55 AA EB 90
		if len(rxBuffer) >= 273 {
			fmt.Println("\n==========================================")
			fmt.Println("   ПОЛУЧЕН ПОЛНЫЙ КАДР ОТ JK BD4A8S4P")
			fmt.Println("==========================================")
			fmt.Println("Первые 32 байта (HEX):", hex.EncodeToString(rxBuffer[:min(32, len(rxBuffer))]))

			parseBasicInfo(rxBuffer)
			fmt.Println("==========================================")
		}
	})

	if err != nil {
		log.Fatalf("Ошибка включения Notifications: %v", err)
	}

	// Отправляем командный запрос кадра
	fmt.Println("Отправка запроса телеметрии в характеристику FFE1...")
	_, err = bmsChar.WriteWithoutResponse(requestFrame)
	if err != nil {
		log.Fatalf("Ошибка записи в FFE1: %v", err)
	}

	// Ждем данные 6 секунд
	time.Sleep(6 * time.Second)
	fmt.Println("\nТестовый прогон завершен.")
}

func parseBasicInfo(frame []byte) {
	i := 11
	for i < len(frame)-4 {
		tag := frame[i]
		switch tag {
		case 0x83: // Total Voltage (2 байта, Big-Endian, unit = 0.01V)
			vRaw := uint16(frame[i+1])<<8 | uint16(frame[i+2])
			fmt.Printf(" Напряжение АКБ : %.2f V\n", float64(vRaw)/100.0)
			i += 3
		case 0x85: // SOC (1 байт, %)
			fmt.Printf(" Заряд (SOC)     : %d%%\n", frame[i+1])
			i += 2
		default:
			i++
		}
	}
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
