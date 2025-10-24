#include "sensor.h"
#include <Arduino.h>
#include <Adafruit_TSL2591.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <time.h>
#include "algo.h"

#define RE_DE  5
#define RXD2   16
#define TXD2   17
#define DHTPIN 14
#define DHTTYPE DHT22

float soilTemperatureRaw = 0.0;
float soilMoistureRaw    = 0.0;
float soilECRaw          = 0.0;
float soilPHRaw          = 0.0;
int   soilNRaw           = 0;
int   soilPRaw           = 0;
int   soilKRaw           = 0;
float humidityRaw = 0.0;
float temperatureRaw = 0.0;
float visibleLuxRaw = 0.0;

float soilTemperature = 0.0;
float soilMoisture    = 0.0;
float soilEC          = 0.0;
float soilPH          = 0.0;
int   soilN           = 0;
int   soilP           = 0;
int   soilK           = 0;
float humidity = 0.0;
float temperature = 0.0;
float visibleLux = 0.0;
uint8_t growthStage = 1;


Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
DHT dht(DHTPIN, DHTTYPE);

extern KalmanState soilMoistureFilter;
extern KalmanState soilTempFilter;
extern KalmanState soilPHFilter;
extern KalmanState soilECFilter;
extern KalmanState soilNFilter;
extern KalmanState soilPFilter;
extern KalmanState soilKFilter;
extern KalmanState airHumidityFilter;
extern KalmanState airTempFilter;
extern KalmanState luxFilter;

void initLuxSensors() {
  if (!tsl.begin()) {
    Serial.println("Không tìm thấy TSL2591 ...");
    // while (1);
  }
  tsl.setGain(TSL2591_GAIN_MED);              // Độ nhạy trung bình
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS); // Thời gian đo 100ms
  Serial.println("[LuxSensor] Initialized.");
}

void initDHTSensors() {
  dht.begin();
  Serial.println("[DHTSensor] Initialized.");
}

void clearUARTBuffer() {
  unsigned long startTime = millis();
  while (Serial2.available() && (millis() - startTime < 1000)) {
    Serial2.read();
    delay(10);
  }
}

void initSoilSensors() {
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);
  delay(50);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (Serial2.available()) Serial2.read();
  // clearUARTBuffer();
  Serial.println("[SoilSensor] Initialized.");
}

void softResetRS485() {
  Serial.println("[RS485] Soft reset...");
  digitalWrite(RE_DE, LOW);
  delay(20);
  Serial2.end();
  delay(20);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(20);
  for (int i=0; i<5; i++){
    Serial2.write(0x00);
    delay(5);
  }
  while (Serial2.available()) Serial2.read();
}

void initSensors() {
  initSoilSensors();
  delay(50);
  initLuxSensors();
  initDHTSensors();
}

uint16_t calculateCRC(uint8_t *frame, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)frame[pos];
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void readSoilSensorRaw() {
  uint8_t request[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00};
  uint8_t response[32]; 
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;
  int index = 0;

  digitalWrite(RE_DE, HIGH);  // Gửi
  delayMicroseconds(100);
  Serial2.write(request, sizeof(request));
  Serial2.flush();
  digitalWrite(RE_DE, LOW);   // Nhận

  unsigned long start = millis();
  while (millis() - start < 200) {
    if (Serial2.available()) {
      response[index++] = Serial2.read();
      if (index >= 19) break;
    }
  }

  if (index < 19) {
    Serial.println("[Sensor] ❌ Không nhận đủ dữ liệu hoặc cảm biến không phản hồi.");
    return;
  }

  //Serial.print("[Sensor] ✔️ Nhận: ");
  // for (int i = 0; i < index; i++) {
  //   Serial.printf("%02X ", response[i]);
  // }
  // Serial.println();

  // Giải mã dữ liệu với chuyển đổi đơn vị chính xác
  soilMoistureRaw    = (response[3] << 8 | response[4]) / 10.0;  // % (0-100)
  soilTemperatureRaw = (response[5] << 8 | response[6]) / 10.0;  // °C
  soilECRaw         = (response[7] << 8 | response[8]) * EC_CONVERSION_FACTOR;  // mS/cm
  soilPHRaw         = (response[9] << 8 | response[10]) / 10.0;  // pH (0-14)
  soilNRaw      = response[11] << 8 | response[12];
  soilPRaw    = response[13] << 8 | response[14];
  soilKRaw    = response[15] << 8 | response[16];

  Serial.printf("[SoilSensor] T: %.1f°C | M: %.1f%% | EC: %.1f | pH: %.1f | NPK: %d-%d-%d\n",
               soilTemperatureRaw, soilMoistureRaw, soilECRaw, soilPHRaw, soilNRaw, soilPRaw, soilKRaw);
}

void readDHTSensorRaw() {
  temperatureRaw = dht.readTemperature();
  humidityRaw    = dht.readHumidity();
  if (isnan(temperatureRaw) || isnan(humidityRaw)) {
    Serial.println("Lỗi đọc DHT22!");
  } else {
    Serial.printf("[DHT22] Temperature: %.1f°C | Humidity: %.1f%%\n", temperatureRaw, humidityRaw);
  }
}

void readLuxSensorRaw() {
  visibleLuxRaw = tsl.getLuminosity(TSL2591_VISIBLE);
  Serial.printf("[TSL2591] -> VisibleLux: %.1f lux\n", visibleLuxRaw);
}

void getSoilSensor() {
  readSoilSensorRaw();
  soilMoisture = kalmanUpdate(soilMoistureFilter, soilMoistureRaw);
  soilTemperature = kalmanUpdate(soilTempFilter, soilTemperatureRaw);
  soilEC = kalmanUpdate(soilECFilter, soilECRaw);
  soilPH = kalmanUpdate(soilPHFilter, soilPHRaw);
  soilN = kalmanUpdate(soilNFilter, soilNRaw);
  soilP = kalmanUpdate(soilPFilter, soilPRaw);
  soilK = kalmanUpdate(soilKFilter, soilKRaw);
  Serial.printf("[SoilSensor-Kalman] T: %.1f°C | M: %.1f%% | EC: %.1f | pH: %.1f | NPK: %d-%d-%d\n",
               soilTemperature, soilMoisture, soilEC, soilPH, soilN, soilP, soilK);
}

void getDHTSensor() {
  readDHTSensorRaw();
  temperature = kalmanUpdate(airTempFilter, temperatureRaw);
  humidity = kalmanUpdate(airHumidityFilter, humidityRaw);
  Serial.printf("[DHT22-Kalman] Temperature: %.1f°C | Humidity: %.1f%%\n", temperature, humidity);
}

void getLuxSensor() {
  readLuxSensorRaw();
  visibleLux = kalmanUpdate(luxFilter, visibleLuxRaw);
  Serial.printf("[TSL2591-Kalman] -> VisibleLux: %.1f lux\n", visibleLux);
}