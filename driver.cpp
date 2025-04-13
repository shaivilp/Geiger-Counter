/*

  _____               _                    _____           _                   _            _       _           
 |  __ \             (_)                  / ____|         | |                 (_)     /\   | |     | |          
 | |__) | __ _____  ___ _ __ ___   __ _  | |     ___ _ __ | |_ __ _ _   _ _ __ _     /  \  | |_ __ | |__   __ _ 
 |  ___/ '__/ _ \ \/ / | '_ ` _ \ / _` | | |    / _ \ '_ \| __/ _` | | | | '__| |   / /\ \ | | '_ \| '_ \ / _` |
 | |   | | | (_) >  <| | | | | | | (_| | | |___|  __/ | | | || (_| | |_| | |  | |  / ____ \| | |_) | | | | (_| |
 |_|   |_|  \___/_/\_\_|_| |_| |_|\__,_|  \_____\___|_| |_|\__\__,_|\__,_|_|  |_| /_/    \_\_| .__/|_| |_|\__,_|
                                                                                             | |                
                                                                                             |_|         

 ESP32 Environmental & Radiation Data Logger
 --------------------------------------------------
 Logs BME280 temperature, pressure, and humidity data
 along with LoRa-received radiation counts from an LND 712 tube.
 Data is stored in an SQLite database on an SD card and sent to
 a Raspberry Pi via UART in JSON format.

 Author: Shaivil Patel
*/

// Required Libraries
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <SQLite.h>

#define BOOST_MODULE_PIN 23
#define SD_CS 4
#define DB_FILE "/sd/data_log.db"

Adafruit_BME280 bme;
SQLiteDB db;

// Use UART1 to send to Raspberry Pi
HardwareSerial piSerial(1);

void initSensors() {
  Wire.begin();
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found!");
    while (1);
  }
}

void initLoRa() {
// SCK, MISO, MOSI, NSS
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(18, 14, 26);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa failed");
    while (1);
  }
}

void initSDandDB() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card mount failed");
    while (1);
  }
  if (!db.open(DB_FILE)) {
    Serial.println("Failed to open database");
    while (1);
  }
  db.exec("CREATE TABLE IF NOT EXISTS logs (timestamp DATETIME DEFAULT CURRENT_TIMESTAMP, radiation TEXT, temperature REAL, pressure REAL, humidity REAL);");
}

void toggleBoostModule() {
  digitalWrite(BOOST_MODULE_PIN, HIGH);
  delay(1000);
  digitalWrite(BOOST_MODULE_PIN, LOW);
  delay(1000);
}

String readLoRa() {
  String radiation = "";
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      radiation += (char)LoRa.read();
    }
  } else {
    radiation = "null";
  }
  return radiation;
}

void readBME280(float &temperature, float &pressure, float &humidity) {
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0;
  humidity = bme.readHumidity();
}

String formatAsJSON(String radiation, float temperature, float pressure, float humidity) {
  String json = "{";
  json += "\"radiation\":\"" + radiation + "\",";
  json += "\"temperature\":" + String(temperature, 2) + ",";
  json += "\"pressure\":" + String(pressure, 2) + ",";
  json += "\"humidity\":" + String(humidity, 2);
  json += "}";
  return json;
}

void sendToSerial(String data) {
  piSerial.println(data);
}

void writeToSQLite(String radiation, float temperature, float pressure, float humidity) {
  String sql = "INSERT INTO logs (radiation, temperature, pressure, humidity) VALUES (";
  sql += "'" + radiation + "', " + String(temperature) + ", " + String(pressure) + ", " + String(humidity) + ");";
  db.exec(sql.c_str());
}

void setup() {
  Serial.begin(115200);
  piSerial.begin(9600, SERIAL_8N1, 17, 16);
  pinMode(BOOST_MODULE_PIN, OUTPUT);
  digitalWrite(BOOST_MODULE_PIN, LOW);

  initSensors();
  initLoRa();
  initSDandDB();
}

void loop() {
  toggleBoostModule();

  String radiation = readLoRa();
  float temperature, pressure, humidity;
  readBME280(temperature, pressure, humidity);

  String json = formatAsJSON(radiation, temperature, pressure, humidity);
  sendToSerial(json);
  writeToSQLite(radiation, temperature, pressure, humidity);
}
