# ESP32 Hardware Configuration

## Microcontroller
- **Board:** ESP32 Dev Board

---

## Libraries Needed (Arduino Library Manager)
**ArduinoJson**  
**Adafruit BME280**  
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

## Boost Module (MOSFET-Switched Power Control)
**Controlled by:** N-channel logic-level MOSFET

| ESP32 → MOSFET | Boost Module Connection |
|----------------|-------------------------|
| GPIO27 → Gate  | -                       |
| GND → Source   | GND of Boost Module     |
| Drain →        | GND of Boost Module     |
| Power Source   | VIN of Boost Module     |

**Notes:**
- Add a 10kΩ pull-down resistor between Gate and Source

---

## SD Card Module
**Interface:** SPI  
**Voltage:** 3.3V

| SD Module Pin | ESP32 Connection |
|---------------|------------------|
| VCC           | 3.3V             |
| GND           | GND              |
| CS            | GPIO4            |
| MOSI          | GPIO23           |
| MISO          | GPIO19           |
| SCK           | GPIO18           |

---

## Geiger-Müller Tube (LND 712) + Pulse Conditioning
**Interface:** Pulse detection (interrupt-driven)  
**Voltage:** High Voltage (~400V on Anode, safe pulse on Cathode)  
**Pulse Read Logic Voltage:** 3.3V (after conditioning)

### Tube Connections:
| LND 712 Terminal | Connection |
|------------------|------------|
| Anode (+)        | HV Boost Output (+400V) via 10MΩ resistor |
| Cathode (-)      | Pulse Conditioner Input (Coupling Capacitor Side) |

### Pulse Conditioner:
- **10MΩ resistor** between HV output and anode (current limiting)
- **15nF coupling capacitor** connected to cathode (blocks DC, passes pulses)
- **100kΩ pull-down resistor** after capacitor to ground (stabilizes pulse)
- Output connected to ESP32 **GPIO32** (can be adjusted if needed)

### ESP32 Connection:
| Pulse Conditioner Output | ESP32 GPIO Pin |
|---------------------------|----------------|
| Conditioned Pulse Output  | GPIO32         |
| GND                       | Common GND     |

**Notes:**
- Interrupt is triggered on **FALLING edge** (pulse is a short negative-going spike)
- ESP32 GND, HV Boost GND, and Pulse Conditioner GND must be **shared (common)**

---

## Serial Connection to Raspberry Pi
**Interface:** USB  
**Baud Rate:** 115200  
**Voltage Logic:** 3.3V

---
