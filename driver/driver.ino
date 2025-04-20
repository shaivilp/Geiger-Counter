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
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include "esp32/rom/ets_sys.h"
#include "soc/gpio_reg.h"
#include "sqlite3.h"


#define BOOST_MODULE_PIN 23
#define BOOST_PIN_MASK (1ULL << BOOST_MODULE_PIN)
#define SD_CS 4
#define DB_FILE "/sd/data_log.db"

Adafruit_BME280 bme;
sqlite3 *db;

// Use UART1 to send to Raspberry Pi
HardwareSerial piSerial(1);

/**
 * @brief This function initializes the BME280 sensor.
 * It uses I2C communication to connect to the sensor.
 * If the sensor is not found, it prints an error message and halts the program.
 * 
 */
void initSensors() {
  Wire.begin();
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found!");
    while (1);
  }
}

/**
 * @brief This function initializes the LoRa module.
 * It sets the pins for SCK, MISO, MOSI, and NSS,
 * and begins communication at 915 MHz.
 * 
 */
void initLoRa() {
  // SCK, MISO, MOSI, NSS
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(18, 14, 26);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa failed");
    while (1);
  }
}

/**
 * @brief This function initializes the SD card and SQLite database.
 * It mounts the SD card and opens the database file. If the database file does not exist,
 * it creates a new one. It also creates a table for logging data if it does not exist.
 */
void initSDandDB() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card mount failed");
    while (1);
  }

  int rc;
  rc = sqlite3_open("/sd/data_log.db", &db);
  if (rc) {
    Serial.printf("Can't open database: %s\n", sqlite3_errmsg(db));
    while (1);
  }
  
  //Create logs table if it does not exist
  const char *createTableSQL = R"(
    CREATE TABLE IF NOT EXISTS logs (
      timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
      radiation TEXT,
      temperature REAL,
      pressure REAL,
      humidity REAL
    );
  )";

  sqlite3_stmt *stmt;
  rc = sqlite3_prepare_v2(db, createTableSQL, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    Serial.printf("Failed to prepare CREATE TABLE statement: %s\n", sqlite3_errmsg(db));
    while (true);
  }
  
  rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    Serial.printf("Failed to execute CREATE TABLE statement: %s\n", sqlite3_errmsg(db));
    while (true);
  }

  sqlite3_finalize(stmt);
}

/** 
 * @brief
 * 
 * This function toggles the boost module on and off at a high frequency.
 * It uses GPIO to control the boost module pin, which is defined as a constant at the top of the file.
 * The function runs in a separate task to avoid blocking the main loop with delayMicroseconds.
 */
void boostToggleTask(void *pvParameters) {
  //Wait 300us before starting the task to allow other tasks to initialize
  delayMicroseconds(300);
  
  //Set the GPIO pin for the boost module as output and set it to LOW
  pinMode(BOOST_MODULE_PIN, OUTPUT);
  digitalWrite(BOOST_MODULE_PIN, LOW);
  const uint32_t mask = BOOST_PIN_MASK;

  while (true) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask);  // ON
    ets_delay_us(10); // ~9.885 µs
    REG_WRITE(GPIO_OUT_W1TC_REG, mask);  // OFF
    ets_delay_us(1); // ~0.9425 µs
  }
}

/**
 * @brief This function reads data from the LoRa module.
 * It checks if there is a packet available and reads the data from it.
 * 
 * @return String The radiation data received from the LoRa module.
 * If no packet is available, it returns "null".
 */
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

/**
 * @brief This function reads data from the BME280 sensor.
 * 
 * @param temperature - Reference to store the temperature value
 * @param pressure  - Reference to store the pressure value
 * @param humidity  - Reference to store the humidity value
 */
void readBME280(float &temperature, float &pressure, float &humidity) {
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0;
  humidity = bme.readHumidity();
}

/**
 * @brief This function writes data to the SQLite database and sends it to the Raspberry Pi.
 * It formats the data as JSON and sends it via UART.
 * 
 * @param radiation - The radiation data received from the LoRa module
 * @param temperature - The temperature data from the BME280 sensor
 * @param pressure  - The pressure data from the BME280 sensor
 * @param humidity  - The humidity data from the BME280 sensor
 */
void writeData(String radiation, float temperature, float pressure, float humidity){
  //Format the data as JSON for sending to the Raspberry Pi
  StaticJsonDocument<256> doc;
  doc["radiation"] = radiation;
  doc["temperature"] = temperature;
  doc["pressure"] = pressure;
  doc["humidity"] = humidity;

  String data;
  serializeJson(doc, data);

  piSerial.println(data);

  //Write the data to the SQLite database
  const char *insertSQL = "INSERT INTO logs (radiation, temperature, pressure, humidity) VALUES (?, ?, ?, ?);";
  sqlite3_stmt *stmt;
  int rc = sqlite3_prepare_v2(db, insertSQL, -1, &stmt, NULL);
  if (rc != SQLITE_OK) {
    Serial.printf("Failed to prepare statement: %s\n", sqlite3_errmsg(db));
    return;
  }

  // Bind values to statement
  sqlite3_bind_text(stmt, 1, radiation.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_double(stmt, 2, temperature);
  sqlite3_bind_double(stmt, 3, pressure);
  sqlite3_bind_double(stmt, 4, humidity);

  // Execute
  rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    Serial.printf("Failed to execute statement: %s\n", sqlite3_errmsg(db));
  }

  sqlite3_finalize(stmt);
}

/**
 * @brief The setup function initializes the serial communication,
 * LoRa module, SD card, and SQLite database.
 * 
 */
void setup() {
  //Initialize Serial and PiSerial
  Serial.begin(115200);
  piSerial.begin(9600, SERIAL_8N1, 17, 16);

  //Initialize sensors, LoRa, and SD card with SQLite database
  initSensors();
  initLoRa();
  initSDandDB();

  //Create separate task for toggling the boost module
  //This is done to avoid blocking the main loop with delayMicroseconds
  xTaskCreatePinnedToCore(boostToggleTask, "BoostToggleTask", 2048, NULL, 2, NULL, 0);
}

/**
 * @brief The loop function continuously reads data from the LoRa module and BME280 sensor,
 * and writes the data to both the SQLite database and Raspberry Pi via UART.
 * 
 */
void loop() {
  String radiation = readLoRa();
  float temperature, pressure, humidity;
  readBME280(temperature, pressure, humidity);

  //Write data to both SQLite and PiSerial
  writeData(radiation, temperature, pressure, humidity);

  //Collect data every 3 seconds
  delay(3000);
}
