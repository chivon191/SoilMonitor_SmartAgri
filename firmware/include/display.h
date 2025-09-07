#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <TFT_eSPI.h>

extern TFT_eSPI tft;

void initTFT();
void drawWidget(int x, int y, int w, int h, String label, float value, String unit, uint16_t bgColor, uint16_t textColor);

#endif