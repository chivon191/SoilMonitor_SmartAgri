#ifndef ALGO_H
#define ALGO_H

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

extern float ALPHA;            // Hệ số Priestley-Taylor
extern float P0;               // Áp suất khí quyển tiêu chuẩn tại mực nước biển (kPa)
extern float TEMP_REF;         // Nhiệt độ tham chiếu (K) để tính áp suất khí quyển
extern float LAPSE_RATE;       // Tỷ lệ giảm nhiệt độ theo độ cao (K/m)
extern float GRAVITY_EXP;      // Số mũ trong công thức tính áp suất khí quyển
extern float GAMMA_CONST;      // Hằng số để tính psychrometric constant (kPa/°C per kPa)
extern float LAMBDA_BASE;      // Nhiệt ẩn hóa hơi cơ bản (MJ/kg, tại 0°C)
extern float LAMBDA_SLOPE;     // Hệ số điều chỉnh nhiệt ẩn hóa hơi theo nhiệt độ (MJ/kg/°C)
extern float ES_CONST;         // Hằng số trong công thức áp suất bão hòa (kPa)
extern float ES_SLOPE;         // Hệ số trong công thức áp suất bão hòa
extern float ES_TEMP_OFFSET;   // Nhiệt độ tham chiếu trong công thức áp suất bão hòa (°C)
extern float DELTA_CONST;      // Hằng số trong công thức tính delta
extern float LUX_TO_RN;        // Hệ số chuyển đổi lux sang MJ/m²/day
extern float GREENHOUSE_FACTOR; // Hệ số điều chỉnh bức xạ trong nhà kính
extern float G;                // Soil heat flux (MJ/m²/day, mặc định 0 cho daily scale)
extern float SM_SENSOR_WEIGHT; // Trọng số của SM_sensor khi hiệu chỉnh SM (30%)
extern float SM_MODEL_WEIGHT;  // Trọng số của SM tính toán (70%)

#endif