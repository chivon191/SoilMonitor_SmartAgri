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