#include "control.h"
#include <Arduino.h>

#define PUMP_PIN  26
#define LED_PIN 27
#define AREA 10.0 // Diện tích vườn (m²)
#define FLOW_RATE 0.1 // Lưu lượng bơm (lít/s)

extern float I_needed;
extern float MIN_I_NEEDED;

void initActuators() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH); // mặc định tắt
  digitalWrite(LED_PIN, HIGH); // mặc định tắt
  Serial.println("[Actuators] Initialized.");
}

void controlIrrigation() {
  if (I_needed > MIN_I_NEEDED) {
    // Kích hoạt bơm tưới bổ sung
    Serial.println("[DEBUG] Start irrigation control");
    // digitalWrite(PUMP_PIN, HIGH);
    // delay(I_needed * AREA * 1000 / FLOW_RATE); // Thời gian bơm (ms)
    // digitalWrite(PUMP_PIN, LOW);
    Serial.printf("Supplemental irrigation with I_needed: %.2f mm\n", I_needed);
    Serial.println("[DEBUG] End irrigation control");
    I_needed = 0; // reset sau khi tưới
  }
}