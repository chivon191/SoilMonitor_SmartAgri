#ifndef ALGO_H
#define ALGO_H
#include <Arduino.h>

// Tham số mô hình Priestley-Taylor
#define ALPHA                   1.26f           // Hệ số Priestley-Taylor
#define P0                      101.3f          // Áp suất khí quyển tiêu chuẩn tại mực nước biển (kPa)
#define TEMP_REF                293.0f          // Nhiệt độ tham chiếu (K) để tính áp suất khí quyển
#define LAPSE_RATE              0.0065f         // Tỷ lệ giảm nhiệt độ theo độ cao (K/m)
#define GRAVITY_EXP             5.26f           // Số mũ trong công thức tính áp suất khí quyển
#define GAMMA_CONST             0.665e-3f       // Hằng số để tính psychrometric constant (kPa/°C per kPa)
#define LAMBDA_BASE             2.501f          // Nhiệt ẩn hóa hơi cơ bản (MJ/kg, tại 0°C)
#define LAMBDA_SLOPE            2.361e-3f       // Hệ số điều chỉnh nhiệt ẩn hóa hơi theo nhiệt độ (MJ/kg/°C)
#define ES_CONST                0.6108f         // Hằng số trong công thức áp suất bão hòa (kPa)
#define ES_SLOPE                17.27f          // Hệ số trong công thức áp suất bão hòa
#define ES_TEMP_OFFSET          237.3f          // Nhiệt độ tham chiếu trong công thức áp suất bão hòa (°C)
#define DELTA_CONST             4098.0f         // Hằng số trong công thức tính delta
#define LUX_TO_RN               0.000198f       // Hệ số chuyển đổi lux sang MJ/m²/day (1 lux ≈ 0.000198 MJ/m²/day)
#define GREENHOUSE_FACTOR       0.7f            // Hệ số điều chỉnh bức xạ trong nhà kính
#define G                       0.0f            // Soil heat flux (MJ/m²/day, mặc định 0 cho daily scale)
#define SM_SENSOR_WEIGHT        0.3f            // Trọng số của SM_sensor khi hiệu chỉnh SM (30%)
#define SM_MODEL_WEIGHT         0.7f            // Trọng số của SM tính toán (70%)
#define MIN_I_NEEDED            3.0f            // Ngưỡng tưới tối thiểu
#define RAW_THRESHOLD           0.7f            // Ngưỡng độ ẩm đất thô

struct KalmanState {
  float x_est;  // Giá trị ước lượng
  float P;      // Sai số dự đoán
  float Q;      // Nhiễu quá trình
  float R;      // Nhiễu đo
  bool initialized; 
};

float kalmanUpdate(KalmanState &state, float measurement);


float calculate_P();
float calculate_gamma();
float calculate_lambda(float T);
float calculate_delta(float T);
float computeET0();
void updateSoilMoisture(bool isScheduledIrrigation);

extern float Kc;
extern float SM;
extern float SM_prev; // mm
extern float D;      // mm/h
extern float I_needed;        // tưới ban đầu
extern float theta_fc;
extern float theta_wp;
extern float Zr; // m
extern float p;
extern float z;

extern float urea_kg_ha;    // kg/ha
extern float urea_g_plant;  // g/cây
extern float dap_kg_ha;
extern float dap_g_plant;  
extern float kcl_kg_ha;
extern float kcl_g_plant;
extern String recommendation_text;
extern bool needs_fertilization;

struct NutrientThreshold {
  float N_low, N_high;
  float P_low, P_high;
  float K_low, K_high;
};

const NutrientThreshold tomatoThreshold = {4.0f, 6.0f, 
                                     0.25f, 0.8f, 
                                     2.5f, 5.0f};

struct StageFactor {
  float urea_factor;
  float dap_factor; 
  float kcl_factor;
  const char* priority_note;
};

StageFactor getStageFactor(int growthStage);
void analyzeNutrientNeeds();

#endif