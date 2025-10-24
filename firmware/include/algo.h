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
#define LUX_TO_RN               0.0000116f      // Hệ số chuyển đổi lux sang MJ/m²/day (1 lux = 0.0000116 MJ/m²/day)
#define EC_CONVERSION_FACTOR    0.001f          // Chuyển đổi µS/cm sang mS/cm
#define MOISTURE_TO_FRACTION    0.01f           // Chuyển đổi % sang fraction (0-1)
#define GREENHOUSE_FACTOR       0.7f            // Hệ số điều chỉnh bức xạ trong nhà kính

/*
 * ========== SMART AGRICULTURE SYSTEM ==========
 * 
 * HỆ THỐNG TƯỚI NƯỚC THÔNG MINH:
 * - Water Balance: Priestley-Taylor + Fuzzy Logic
 * - Decision Table: pH + EC analysis
 * - Smart Irrigation: Weather-based decisions
 * - Leaching System: EC management
 * 
 * HỆ THỐNG BÓN PHÂN THÔNG MINH:
 * - NPK Analysis: 3-day historical data
 * - Reference Garden: High-yield comparison
 * - Fertilizer Schedule: 6-month planning
 * - Smart Recommendations: Deficit-based calculation
 * 
 * ĐƠN VỊ ĐO CHÍNH XÁC:
 * - Soil: % → fraction, µS/cm → mS/cm
 * - Water: mm/day, fraction (0-1)
 * - NPK: mg/kg, kg/ha, g/plant
 * - Time: Unix timestamp, intervals
 * 
 * NGƯỠNG THRESHOLD:
 * - EC: 2.5/3.0/4.0 mS/cm
 * - pH: 5.5/6.0-6.5/7.5
 * - Moisture: 75% soil, 80% air
 * - Temperature: 15°C soil minimum
 */
#define G                       0.0f            // Soil heat flux (MJ/m²/day, mặc định 0 cho daily scale)
#define SM_SENSOR_WEIGHT        0.3f            // Trọng số của SM_sensor khi hiệu chỉnh SM (30%)
#define SM_MODEL_WEIGHT         0.7f            // Trọng số của SM tính toán (70%)
#define MIN_I_NEEDED            3.0f            // Ngưỡng tưới tối thiểu
#define RAW_THRESHOLD           0.7f            // Ngưỡng độ ẩm đất thô
#define EC_IRRIGATION           0.8f            // EC nước tưới (mS/cm)
#define EC_TARGET               2.0f            // EC mục tiêu (mS/cm)
#define EC_THRESHOLD            2.5f            // Ngưỡng EC cao (mS/cm)
#define DRAINAGE_FACTOR         0.15f
#define HIGH_HUMIDITY_THRESHOLD 80.0f    // Ngưỡng độ ẩm không khí cao (%, không tưới)
#define LOW_SOIL_TEMP_THRESHOLD 15.0f    // Ngưỡng nhiệt độ đất thấp (°C, không tưới)
#define HIGH_SOIL_MOISTURE_THRESHOLD 75.0f  // Ngưỡng độ ẩm đất cao (%, không tưới)
#define WATER_SAVING_FACTOR     0.7f     // Hệ số tiết kiệm nước
#define LEACHING_FRACTION       0.2f     // Tỷ lệ nước tưới dùng để xả mặn
#define EC_CRITICAL_THRESHOLD   3.0f     // Ngưỡng EC nguy hiểm (mS/cm)
#define EC_EMERGENCY_THRESHOLD  4.0f     // Ngưỡng EC khẩn cấp (mS/cm)

// Ngưỡng pH cho Decision Table (đơn vị: pH)
#define PH_ACID_THRESHOLD       5.5f     // pH < 5.5: Quá chua
#define PH_OPTIMAL_LOW         6.0f     // pH tối ưu thấp
#define PH_OPTIMAL_HIGH        6.5f     // pH tối ưu cao  
#define PH_ALKALINE_THRESHOLD  7.5f     // pH > 7.5: Quá kiềm

// Ngưỡng EC cho Decision Table (đơn vị: mS/cm)
#define EC_DECISION_LOW        2.5f     // EC < 2.5: Bình thường
#define EC_DECISION_HIGH       3.0f     // EC 2.5-3.0: Cao
#define EC_DECISION_CRITICAL   4.0f     // EC > 4.0: Khẩn cấp

// Fuzzy Logic parameters
#define FUZZY_OVERLAP           0.3f     // Độ chồng lấp fuzzy (0.3 pH units)
#define FUZZY_STRONG            0.8f     // Ngưỡng fuzzy "mạnh"
#define FUZZY_MODERATE          0.5f     // Ngưỡng fuzzy "vừa phải"
#define FUZZY_WEAK              0.2f     // Ngưỡng fuzzy "yếu"

// Hệ thống bón phân thông minh
#define NPK_HISTORY_DAYS        3        // Số ngày lưu trữ dữ liệu NPK
#define FERTILIZER_INTERVAL     14       // Chu kỳ bón phân (ngày)
#define PLANT_DENSITY          25000     // Mật độ cây/ha
#define ABSORPTION_RATE_N       0.7f     // Tỷ lệ hấp thụ N
#define ABSORPTION_RATE_P       0.5f     // Tỷ lệ hấp thụ P  
#define ABSORPTION_RATE_K       0.8f     // Tỷ lệ hấp thụ K
#define FERTILIZER_EFFICIENCY   0.8f     // Hiệu suất phân bón

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

struct KalmanState {
  float x_est;  // Giá trị ước lượng
  float P;      // Sai số dự đoán
  float Q;      // Nhiễu quá trình
  float R;      // Nhiễu đo
  bool initialized; 
};


struct StageFactor {
  float urea_factor;
  float dap_factor; 
  float kcl_factor;
  const char* priority_note;
};

// Cấu trúc cho Decision Table
struct DecisionRule {
  float ph_min, ph_max;
  float ec_min, ec_max;
  const char* action;
  const char* recommendation;
  int priority;  // 1=cao, 2=trung bình, 3=thấp
};

// Cấu trúc cho Fuzzy Logic
struct FuzzyMembership {
  float very_acid;    // pH < 5.5
  float acid;         // pH 5.5-6.0
  float neutral;      // pH 6.0-6.5
  float alkaline;     // pH 6.5-7.5
  float very_alkaline; // pH > 7.5
};

// Cấu trúc khuyến nghị tổng hợp
struct SoilRecommendation {
  String ph_status;
  String ec_status;
  String primary_action;
  String secondary_action;
  float urgency_level;  // 0-1
  String detailed_advice;
};

// Cấu trúc dữ liệu NPK lịch sử
struct NPKData {
  float N, P, K;        // mg/kg
  unsigned long timestamp;  // Unix timestamp
  bool valid;           // Dữ liệu hợp lệ
};

// Cấu trúc vườn mẫu năng suất cao
struct ReferenceGarden {
  float N_optimal;      // mg/kg
  float P_optimal;      // mg/kg  
  float K_optimal;      // mg/kg
  float yield_kg_ha;    // kg/ha
  String crop_type;     // Loại cây trồng
  String season;        // Mùa vụ
};

// Cấu trúc khuyến nghị bón phân
struct FertilizerRecommendation {
  float N_deficit;      // mg/kg thiếu
  float P_deficit;      // mg/kg thiếu
  float K_deficit;      // mg/kg thiếu
  float urea_kg_ha;     // kg/ha
  float dap_kg_ha;      // kg/ha
  float kcl_kg_ha;      // kg/ha
  float urea_g_plant;   // g/cây
  float dap_g_plant;    // g/cây
  float kcl_g_plant;    // g/cây
  String schedule_text; // Lịch bón phân
  bool needs_fertilizer; // Cần bón phân
};

// Cấu trúc lịch bón phân 6 tháng
struct FertilizerSchedule {
  String month;
  int week;
  String fertilizer_type;
  float amount_kg_ha;
  float amount_g_plant;
  String application_method;
  String notes;
};

float kalmanUpdate(KalmanState &state, float measurement);
float calculate_P();
float calculate_gamma();
float calculate_lambda(float T);
float calculate_delta(float T);
float computeET0();
float computeLF();
float computeDrainage(float SM_prev, float TAW);
void updateSoilMoisture(bool isScheduledIrrigation);

// Hàm mới cho tưới thông minh
bool shouldIrrigate();
float getGrowthStageFactor(int growthStage);
float getWaterSavingFactor();
bool isWaterSavingMode();
float computeETcImproved();

// Hàm xử lý EC cao (đất mặn)
float computeLeachingFraction();
float computeLeachingWater(float baseIrrigation);
bool isECCritical();
bool isECEmergency();
String getECStatus();
void handleHighEC();

// Hàm Decision Table + Fuzzy Logic
FuzzyMembership calculateFuzzyMembership(float ph);
DecisionRule findMatchingRule(float ph, float ec);
SoilRecommendation generateSoilRecommendation();
String getPHStatus();
String getECStatusDetailed();

// Hệ thống bón phân thông minh
void storeNPKHistory(float N, float P, float K);
NPKData getAverageNPK();
ReferenceGarden getReferenceGarden();
FertilizerRecommendation calculateFertilizerNeeds();
FertilizerSchedule* generate6MonthSchedule();
String formatFertilizerSchedule();
void printFertilizerAnalysis();

#endif