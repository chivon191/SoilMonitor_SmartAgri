#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

void initSensors();
void initLuxSensors();
void initDHTSensors();
void clearUARTBuffer();
void initSoilSensors();
void readSoilSensorRaw();
void readDHTSensorRaw();
void readLuxSensorRaw();
void getSoilSensor();
void getDHTSensor();
void getLuxSensor();

extern float soilTemperatureRaw;
extern float soilMoistureRaw;
extern float soilECRaw;
extern float soilPHRaw;
extern int soilNRaw;
extern int soilPRaw;
extern int soilKRaw;
extern float humidityRaw;
extern float temperatureRaw;
extern float visibleLuxRaw;

extern float soilTemperature;
extern float soilMoisture;
extern float soilEC;
extern float soilPH;
extern int soilN;
extern int soilP;
extern int soilK;
extern float humidity;
extern float temperature;
extern float visibleLux;
extern uint8_t growthStage;

#endif