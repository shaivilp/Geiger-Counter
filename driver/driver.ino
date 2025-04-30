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
 along with radiation counts from an LND 712 tube.
 Data is stored in an SQLite database on an SD card and sent to
 a Raspberry Pi via UART in JSON format.

 Author: Shaivil Patel
*/

// Required Libraries
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "esp32/rom/ets_sys.h"
#include "soc/gpio_reg.h"
#include "sqlite3.h"

#define GEIGER_PIN 32 
#define BOOST_MODULE_PIN 27
#define BOOST_PIN_MASK (1ULL << BOOST_MODULE_PIN)
#define SD_CS 4
#define DB_FILE "/sd/data_log.db"

Adafruit_BME280 bme;
sqlite3 *db;
volatile uint32_t pulseCount = 0;

/**
 * @brief This function initializes the BME280 sensor.
 * It uses I2C communication to connect to the sensor.
 * If the sensor is not found, it prints an error message and halts the program.
 * 
 */
void initSensors() {
  if (!bme.begin(0x76)) {
    if(!bme.begin(0x77)){
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    return;
    }
  }

  Serial.println("Successfully connected to the BME280 sensor!");
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
  rc = sqlite3_open(DB_FILE, &db);
  if (rc) {
    Serial.printf("Can't open database: %s\n", sqlite3_errmsg(db));
    while (1);
  }
  
  //Create logs table if it does not exist
  const char *createTableSQL = R"(
    CREATE TABLE IF NOT EXISTS logs (
      timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
      radiation INT,
      temperature REAL,
      pressure REAL,
      humidity REAL
    );
  )";

  sqlite3_stmt *stmt;
  rc = sqlite3_prepare_v2(db, createTableSQL, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    Serial.printf("Failed to prepare CREATE TABLE statement: %s\n", sqlite3_errmsg(db));
  }
  
  rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    Serial.printf("Failed to execute CREATE TABLE statement: %s\n", sqlite3_errmsg(db));
  }

  sqlite3_finalize(stmt);
}

/**
 * @brief This function is called when a radiation pulse is detected.
 * It increments the pulseCount variable to keep track of the number of pulses.
 */
void IRAM_ATTR onRadiationPulse() {
  pulseCount++;
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
  const uint32_t mask = BOOST_PIN_MASK;

  while (true) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask);  // ON
    ets_delay_us(10); // ~9.885 µs
    REG_WRITE(GPIO_OUT_W1TC_REG, mask);  // OFF
    ets_delay_us(1); // ~0.9425 µs
  }
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
void writeData(int radiation, float temperature, float pressure, float humidity){
  //Format the data as JSON for sending to the Raspberry Pi
  StaticJsonDocument<256> doc;
  doc["radiation"] = radiation;
  doc["temperature"] = temperature;
  doc["pressure"] = pressure;
  doc["humidity"] = humidity;

  String data;
  serializeJson(doc, data);

  Serial.println(data);

  //Write the data to the SQLite database
  const char *insertSQL = "INSERT INTO logs (radiation, temperature, pressure, humidity) VALUES (?, ?, ?, ?);";
  sqlite3_stmt *stmt;
  int rc = sqlite3_prepare_v2(db, insertSQL, -1, &stmt, NULL);
  if (rc != SQLITE_OK) {
    Serial.printf("Failed to prepare statement: %s\n", sqlite3_errmsg(db));
    return;
  }

  // Bind values to statement
  sqlite3_bind_int(stmt, 1, radiation);
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
  //Initialize Serial
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting setup...");

  Wire.begin(21, 22);

  //Set the GPIO pin for the boost module as output and set it to LOW
  pinMode(BOOST_MODULE_PIN, OUTPUT);
  digitalWrite(BOOST_MODULE_PIN, LOW);

  // Setup Geiger pulse pin
  pinMode(GEIGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), onRadiationPulse, FALLING);

  //Initialize sensors, LoRa, and SD card with SQLite database
  initSensors();
  initSDandDB();

  //Create separate task for toggling the boost module
  //This is done to avoid blocking the main loop with delayMicroseconds
  //xTaskCreatePinnedToCore(boostToggleTask, "BoostToggleTask", 2048, NULL, 2, NULL, 0);
  Serial.println("Finished setup.");
}

/**
 * @brief The loop function continuously reads data from the LoRa module and BME280 sensor,
 * and writes the data to both the SQLite database and Raspberry Pi via UART.
 * 
 */
void loop() {
  static uint32_t lastPulseCount = 0;
  float temperature, pressure, humidity;

  // Read BME280 sensor data
  readBME280(temperature, pressure, humidity);

  //Calculate CPM from pulse count
  uint32_t currentCount = pulseCount;
  uint32_t delta = currentCount - lastPulseCount;
  lastPulseCount = currentCount;

  // 3s interval → scale by 20 for CPM
  uint32_t cpm = delta * 20;

  // Serial.println("---- BME280 Readings ----");
  // Serial.printf("Temperature: %.2f °C\n", temperature);
  // Serial.printf("Pressure:    %.2f hPa\n", pressure);
  // Serial.printf("Humidity:    %.2f %%\n", humidity);
  // Serial.println("-------------------------");

  //Write data to both SQLite and PiSerial
  writeData(cpm, temperature, pressure, humidity);

  //Collect data every 3 seconds
  delay(3000);
}
