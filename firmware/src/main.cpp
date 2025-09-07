#include <Arduino.h>
#include "sensor.h"
#include "control.h"
#include "display.h"
#include "algo.h"
#include <PubSubClient.h>
#include <WiFi.h>

const char* ssid = "donnn";
const char* password = "88888888";

// float latitude = 10.827263;
// float longitude = 106.630918;

unsigned long lastSensorTime = 0;
unsigned long lastLogTime = 0;

const unsigned long logInterval    = 5 * 60 * 1000; // 5 phút gửi Firebase
int IRRIGATION_HOUR_1 = 5;
int IRRIGATION_HOUR_2 = 17;
int UPDATE_INTERVAL = 60000; //test 1 phút đọc cảm biến
float WIFI_RECONNECT_INTERVAL = 300000; //test 5 phút thử reconnect wifi
bool isScheduledIrrigation = false;
int year;
int month;
int day;
int hour;
int minute;

void initTime() {
  // Cấu hình NTP
  configTime(7*3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7
  Serial.print("Waiting for time");
  while (time(nullptr) < 100000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Time OK");
}

void getCurrentDateTime() {
  time_t now;
  struct tm timeinfo;

  time(&now);
  gmtime_r(&now, &timeinfo); // Lấy giờ UTC

  year  = timeinfo.tm_year + 1900;
  month = timeinfo.tm_mon + 1;
  day   = timeinfo.tm_mday;
  hour  = (timeinfo.tm_hour + 7) % 24;
  minute= timeinfo.tm_min;
}

void reconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Reconnecting to WiFi");
        WiFi.reconnect();
        for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("WiFi reconnected");
            initTime(); // Khởi tạo lại NTP khi kết nối thành công
        } else {
            Serial.println("WiFi reconnect failed");
        }
    }
}

void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  reconnectWiFi();
  initSensors();
  initTFT();
  initActuators();

  Serial.println("[System] Setup completed");
}

void sensorCycle() {
  getSoilSensor();
  getDHTSensor();
  getLuxSensor();
  drawWidget(5, 35, 150, 60, "Humidity", soilMoisture, "%", TFT_BLUE, TFT_WHITE);
  drawWidget(165, 35, 150, 60, "Temperature", soilTemperature, "C", TFT_RED, TFT_WHITE);
  drawWidget(5, 105, 150, 60, "EC", soilEC, "uS/cm", TFT_GREEN, TFT_WHITE);
  drawWidget(165, 105, 150, 60, "pH", soilPH, "", TFT_CYAN, TFT_WHITE);
  drawWidget(5, 175, 100, 60, "Nitro", soilN, "mg/kg", TFT_YELLOW, TFT_BLACK);
  drawWidget(110, 175, 100, 60, "Photpho", soilP, "mg/kg", TFT_MAGENTA, TFT_WHITE);
  drawWidget(215, 175, 100, 60, "Kali", soilK, "mg/kg", TFT_ORANGE, TFT_WHITE);
}

// Hàm kiểm tra thời gian tưới cố định
bool isScheduledIrrigationTime() {
  if (WiFi.status() != WL_CONNECTED) {
    return false; // Không tưới cố định nếu mất WiFi
  }
  getCurrentDateTime();
  // Tưới trong khoảng ±5 phút của 6h và 16h
  return (hour == IRRIGATION_HOUR_1 && minute < 5) || (hour == IRRIGATION_HOUR_2 && minute < 5);
}

void loop() {
  static uint32_t lastUpdate = 0;
  static uint32_t lastWiFiReconnect = 0;

  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiReconnect >= WIFI_RECONNECT_INTERVAL) {
    reconnectWiFi();
    lastWiFiReconnect = millis();
  }

  static int lastYear = -1, lastMonth = -1, lastDay = -1, lastHour = -1, lastMinute = -1;
  if (isScheduledIrrigationTime()) {
    if (minute != lastMinute || hour != lastHour || day != lastDay || month != lastMonth || year != lastYear) {
      sensorCycle();
      updateSoilMoisture(true);
      lastHour = hour;
      lastMinute = minute;
      lastDay = day;
      lastMonth = month;
      lastYear = year;
    }
  }
  // // Kiểm tra thời gian tưới cố định
  // if (isScheduledIrrigationTime()) {
  //   sensorCycle();
  //   updateSoilMoisture(true);  // Tưới cố định
  // }

  // Kiểm tra water balance mỗi 2 giờ
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    sensorCycle();
    updateSoilMoisture(false);  // Kiểm tra bổ sung
    lastUpdate = millis();
  }

  if (I_needed > 0) {
    controlIrrigation();
  }

  delay(10);
}