# ESP32 Data Logger Hardware Configuration


## Microcontroller
- **Board:** ESP32 Dev Board


---
## Libraries Needed (Arduino Library Manager)
**Adafruit BME280**
**LoRa by Sandeep Mistry**
**SD.h**
[**SQLite_Arduino**](https://github.com/siara-cc/SQLite_Arduino)

---

## BME280 Sensor (Temperature, Humidity, Pressure)
**Interface:** I2C  
**Voltage:** 3.3V

| BME280 Pin | ESP32 Connection |
|------------|------------------|
| VIN        | 3.3V             |
| GND        | GND              |
| SDA        | GPIO21           |
| SCL        | GPIO22           |

---

## LoRa Module (Receiving Radiation Data)
**Module Type:** SX1276/SX1278  
**Interface:** SPI  
**Voltage:** 3.3V

| LoRa Pin | ESP32 Connection |
|----------|------------------|
| VCC      | 3.3V             |
| GND      | GND              |
| SCK      | GPIO5            |
| MISO     | GPIO19           |
| MOSI     | GPIO27           |
| NSS/CS   | GPIO18           |
| RESET    | GPIO14           |
| DIO0     | GPIO26           |

---

## Boost Module (MOSFET-Switched Power Control)
**Controlled by:** N-channel logic-level MOSFET (e.g., IRLZ34N, AO3400)

| ESP32 → MOSFET | Boost Module Connection |
|----------------|-------------------------|
| GPIO23 → Gate  | -                       |
| GND → Source   | GND of Boost Module     |
| Drain →        | GND of Boost Module     |
| Power Source   | VIN of Boost Module     |

**Notes:**
- Add a 10kΩ pull-down resistor between Gate and Source
---

## SD Card Module
**Interface:** SPI  
**Voltage:** 3.3V (or 5V with logic level shifting)

| SD Module Pin | ESP32 Connection |
|---------------|------------------|
| VCC           | 3.3V or 5V       |
| GND           | GND              |
| CS            | GPIO4            |
| MOSI          | GPIO23           |
| MISO          | GPIO19           |
| SCK           | GPIO18           |

**Note:** If using both SD and LoRa (SPI), ensure separate CS pins and manage selection in code.

---

## Serial Connection to Raspberry Pi
**Interface:** UART  
**Baud Rate:** 9600  
**Voltage Logic:** 3.3V (direct connection safe)

| ESP32 Pin | Raspberry Pi Connection |
|-----------|--------------------------|
| GPIO17 (TX) | GPIO15 (RXD)           |
| GND         | GND                    |

**Note:** Do not connect ESP32 RX to Pi TX unless bidirectional communication is required.

---
