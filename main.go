package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"strings"
	"time"

	"tinygo.org/x/bluetooth"
)

const TargetBMSName = "60126310241"
const TargetBMSAddress = "C8:47:80:53:5E:04"

var (
	serviceUUID, _ = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	charUUID, _    = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
)

var headerResponseMagic = []byte{0x55, 0xAA, 0xEB, 0x90}

type BMSData struct {
	DeviceModel  string
	HardwareVer  string
	SoftwareVer  string
	TotalVoltage float64
	Current      float64
	SOC          uint8
	TempMOS      float64
	Temp1        float64
	Temp2        float64
	Cells        []float64
	HasTelemetry bool
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

		if name == TargetBMSName || strings.EqualFold(addr, TargetBMSAddress) {
			fmt.Printf("[УСПЕХ] Найдена JK BMS! Name: %s | Address: %s\n", name, addr)
			targetDevice = result
			found = true
			adapter.StopScan()
		}
	})

	if err != nil || !found {
		log.Fatalf("Устройство не найдено.")
	}

	fmt.Printf("Подключение к %s...\n", targetDevice.Address.String())
	device, err := adapter.Connect(targetDevice.Address, bluetooth.ConnectionParams{})
	if err != nil {
		log.Fatalf("Не удалось подключиться: %v", err)
	}
	defer device.Disconnect()

	services, err := device.DiscoverServices([]bluetooth.UUID{serviceUUID})
	if err != nil || len(services) == 0 {
		log.Fatalf("Сервис FFE0 не найден: %v", err)
	}

	chars, err := services[0].DiscoverCharacteristics([]bluetooth.UUID{charUUID})
	if err != nil || len(chars) == 0 {
		log.Fatalf("Характеристика FFE1 не найдена: %v", err)
	}

	bmsChar := chars[0]
	rxBuffer := make([]byte, 0, 2048)
	var finalData BMSData

	err = bmsChar.EnableNotifications(func(buf []byte) {
		rxBuffer = append(rxBuffer, buf...)
		rxBuffer = processJKBuffer(rxBuffer, &finalData)
	})

	if err != nil {
		log.Fatalf("Ошибка подписки на уведомления: %v", err)
	}

	time.Sleep(500 * time.Millisecond)

	// 1. Сначала запрашиваем Device Info (0x97)
	cmd97 := buildJKCommand(0x97)
	fmt.Println("Отправка запроса Device Info (0x97)...")
	if _, err := bmsChar.WriteWithoutResponse(cmd97); err != nil {
		log.Fatalf("Ошибка отправки 0x97: %v", err)
	}

	time.Sleep(700 * time.Millisecond)

	// 2. Затем запрашиваем Status/Telemetry (0x96) с повтором при необходимости
	for attempt := 1; attempt <= 8; attempt++ {
		cmd96 := buildJKCommand(0x96)
		fmt.Printf("Отправка запроса Status (0x96), попытка %d/8...\n", attempt)
		if _, err := bmsChar.WriteWithoutResponse(cmd96); err != nil {
			log.Printf("Ошибка отправки 0x96: %v", err)
		}

		for i := 0; i < 10; i++ {
			if finalData.HasTelemetry {
				break
			}
			time.Sleep(100 * time.Millisecond)
		}

		if finalData.HasTelemetry {
			break
		}
	}

	if finalData.HasTelemetry || finalData.DeviceModel != "" {
		printBMSReport(finalData)
	} else {
		fmt.Printf("\n[!] Ответ не получен. Накоплено байт в буфере: %d\n", len(rxBuffer))
	}

	fmt.Println("\nТест завершён.")
}

func buildJKCommand(cmdType byte) []byte {
	cmd := make([]byte, 20)
	cmd[0] = 0xAA
	cmd[1] = 0x55
	cmd[2] = 0x90
	cmd[3] = 0xEB
	cmd[4] = cmdType

	var sum byte = 0
	for i := 0; i < 19; i++ {
		sum += cmd[i]
	}
	cmd[19] = sum
	return cmd
}

func processJKBuffer(buf []byte, outData *BMSData) []byte {
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

		if crc != frame[299] {
			log.Printf("[RX] Ошибка CRC: calculated=0x%02X received=0x%02X", crc, frame[299])
			buf = buf[1:]
			continue
		}

		frameType := frame[4]
		counter := frame[5]

		log.Printf("[RX] Frame 0x%02X, counter=0x%02X, CRC OK", frameType, counter)

		switch frameType {
		case 0x01:
			log.Printf("[RX] Получен Frame 0x01 (settings)")

		case 0x02:
			parseTelemetryFrame(frame, outData)
			outData.HasTelemetry = true
			log.Printf("[RX] Telemetry: %.3f V, %.3f A, %d%%, %d cells",
				outData.TotalVoltage, outData.Current, outData.SOC, len(outData.Cells))

		case 0x03:
			parseInfoFrame(frame, outData)

		default:
			log.Printf("[RX] Неизвестный Frame type: 0x%02X", frameType)
		}

		buf = buf[frameSize:]
	}
}

func parseInfoFrame(frame []byte, data *BMSData) {
	if len(frame) < 38 {
		return
	}

	data.DeviceModel = strings.Trim(string(frame[6:22]), "\x00 ")
	data.HardwareVer = strings.Trim(string(frame[22:30]), "\x00 ")
	data.SoftwareVer = strings.Trim(string(frame[30:38]), "\x00 ")

	log.Printf("[INFO] Model=%s HW=%s SW=%s", data.DeviceModel, data.HardwareVer, data.SoftwareVer)
}

func parseTelemetryFrame(frame []byte, data *BMSData) {
	if len(frame) < 300 {
		return
	}

	// 1. Напряжения ячеек (8 штук, начиная с индекса 6)
	cellCount := 8
	data.Cells = make([]float64, 0, cellCount)
	for i := 0; i < cellCount; i++ {
		pos := 6 + (i * 2)
		raw := binary.LittleEndian.Uint16(frame[pos : pos+2])
		data.Cells = append(data.Cells, float64(raw)*0.001)
	}

	// 2. Общие параметры (напряжение и ток)
	rawVoltage := binary.LittleEndian.Uint32(frame[118+32 : 122+32])
	data.TotalVoltage = float64(rawVoltage) * 0.001

	rawCurrent := int32(binary.LittleEndian.Uint32(frame[126+32 : 130+32]))
	data.Current = float64(rawCurrent) * 0.001

	// 3. Точные температуры по индексам из нового дампа
	rawMOS := int16(binary.LittleEndian.Uint16(frame[144:146]))
	rawT1 := int16(binary.LittleEndian.Uint16(frame[162:164]))
	rawT2 := int16(binary.LittleEndian.Uint16(frame[164:166]))

	data.TempMOS = float64(rawMOS) * 0.1
	data.Temp1 = float64(rawT1) * 0.1
	data.Temp2 = float64(rawT2) * 0.1

	// 4. SOC %
	if len(frame) > 173 {
		data.SOC = frame[173]
	}

}

func printBMSReport(d BMSData) {
	fmt.Println("\n==========================================")
	fmt.Println("       ПОКАЗАТЕЛИ JK BD4A8S4P (Go)")
	fmt.Println("==========================================")
	if d.DeviceModel != "" {
		fmt.Printf(" Модель:            %s\n", d.DeviceModel)
		fmt.Printf(" Прошивка:          %s (HW: %s)\n", d.SoftwareVer, d.HardwareVer)
		fmt.Println("------------------------------------------")
	}
	fmt.Printf(" Напряжение сборки: %.2f V\n", d.TotalVoltage)
	fmt.Printf(" Ток:               %.2f A\n", d.Current)
	fmt.Printf(" Уровень заряда:    %d%%\n", d.SOC)
	fmt.Printf(" Мощность:          %.1f W\n", d.TotalVoltage*d.Current)
	fmt.Println("------------------------------------------")
	fmt.Printf(" Температура MOS:   %.1f °C\n", d.TempMOS)
	fmt.Printf(" Температура T1:    %.1f °C\n", d.Temp1)
	fmt.Printf(" Температура T2:    %.1f °C\n", d.Temp2)
	fmt.Println("------------------------------------------")
	fmt.Println(" Ячейки (V):")
	if len(d.Cells) == 0 {
		fmt.Println("   [Нет данных по ячейкам]")
	}
	for idx, v := range d.Cells {
		fmt.Printf("   Ячейка %d: %.3f V\n", idx+1, v)
	}
	fmt.Println("==========================================")
}
