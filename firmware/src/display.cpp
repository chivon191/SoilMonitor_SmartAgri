#include "display.h"

TFT_eSPI tft = TFT_eSPI();

void initTFT() {
    tft.init();
    tft.setRotation(1); // Xoay ngang (320x240)
    tft.fillScreen(TFT_BLACK); // Xóa màn hình
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextFont(4);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("NPK Dashboard", 160, 10); // Tiêu đề
    Serial.println("[TFT Screen] Initialized.");
}

void drawWidget(int x, int y, int w, int h, String label, float value, String unit, uint16_t bgColor, uint16_t textColor) {
    tft.fillRect(x, y, w, h, bgColor); // Vẽ nền widget
    tft.drawRect(x, y, w, h, TFT_WHITE); // Vẽ viền
    tft.setTextColor(textColor, bgColor);
    tft.setTextSize(1);
    tft.setTextFont(2); // Font nhỏ cho nhãn
    tft.setTextDatum(TL_DATUM);
    tft.drawString(label, x + 5, y + 5);
    tft.setTextFont(4); // Font lớn cho giá trị
    tft.setTextDatum(MC_DATUM);
    tft.drawFloat(value, 1, x + w/2, y + h/2); // Hiển thị giá trị với 1 số thập phân
    tft.setTextFont(2); // Font nhỏ cho đơn vị
    tft.drawString(unit, x + w/2, y + h - 15);
}