#include "algo.h"
#include "sensor.h"
#include <Arduino.h>

// Biến toàn cục từ cảm biến
extern float temperature;      // °C
extern float humidity;         // % (độ ẩm tương đối)
extern float visibleLux;    // lux
extern float soilMoisture;     // fraction (0-1)
extern float soilTemperature;  // °C

// Tham số cho cà chua trong nhà kính
float Kc = 0.8;               // Hệ số cây trồng (0.6-1.2 tùy giai đoạn)
float SM;                     // Độ ẩm đất hiện tại (mm)
float SM_prev;                // Độ ẩm đất trước đó (mm)
float D = 12.0;               // Drainage (mm/day, tương đương 0.5 mm/h * 24)
float I_needed = 0;           // Lượng nước cần tưới (mm/day)
float theta_fc = 0.35;        // Field Capacity (fraction, đất sạch trộn sẵn)
float theta_wp = 0.15;        // Wilting Point (fraction)
float Zr = 0.3;               // Độ sâu rễ (m)
float p = 0.5;                // Management Allowed Depletion (phù hợp cho cà chua)
float z = 100;                // Độ cao ước lượng (mét)

// Tham số mô hình Priestley-Taylor
float ALPHA = 1.26;           // Hệ số Priestley-Taylor
float P0 = 101.3;             // Áp suất khí quyển tiêu chuẩn tại mực nước biển (kPa)
float TEMP_REF = 293;         // Nhiệt độ tham chiếu (K) để tính áp suất khí quyển
float LAPSE_RATE = 0.0065;    // Tỷ lệ giảm nhiệt độ theo độ cao (K/m)
float GRAVITY_EXP = 5.26;     // Số mũ trong công thức tính áp suất khí quyển
float GAMMA_CONST = 0.665e-3; // Hằng số để tính psychrometric constant (kPa/°C per kPa)
float LAMBDA_BASE = 2.501;    // Nhiệt ẩn hóa hơi cơ bản (MJ/kg, tại 0°C)
float LAMBDA_SLOPE = 2.361e-3;// Hệ số điều chỉnh nhiệt ẩn hóa hơi theo nhiệt độ (MJ/kg/°C)
float ES_CONST = 0.6108;      // Hằng số trong công thức áp suất bão hòa (kPa)
float ES_SLOPE = 17.27;       // Hệ số trong công thức áp suất bão hòa
float ES_TEMP_OFFSET = 237.3; // Nhiệt độ tham chiếu trong công thức áp suất bão hòa (°C)
float DELTA_CONST = 4098;     // Hằng số trong công thức tính delta
float LUX_TO_RN = 0.000198;   // Hệ số chuyển đổi lux sang MJ/m²/day (1 lux ≈ 0.000198 MJ/m²/day)
float GREENHOUSE_FACTOR = 0.7; // Hệ số điều chỉnh bức xạ trong nhà kính
float G = 0;                  // Soil heat flux (MJ/m²/day, mặc định 0 cho daily scale)
float SM_SENSOR_WEIGHT = 0.3; // Trọng số của SM_sensor khi hiệu chỉnh SM (30%)
float SM_MODEL_WEIGHT = 0.7;  // Trọng số của SM tính toán (70%)
float MIN_I_NEEDED = 3.0;
float RAW_THRESHOLD = 0.7;

// Soil sensors
KalmanState soilMoistureFilter   = {0, 1.0, 0.05, 2.0, false};   // soil moisture (%)
KalmanState soilTempFilter       = {0, 1.0, 0.01, 0.5, false};   // soil temperature (°C)
KalmanState soilPHFilter         = {0,  1.0, 0.01, 0.2, false};   // pH
KalmanState soilECFilter         = {0, 1.0, 0.05, 5.0, false};   // EC (µS/cm)
KalmanState soilNFilter          = {0, 1.0, 0.05, 2.0, false};   // Nitrogen (mg/kg)
KalmanState soilPFilter          = {0, 1.0, 0.05, 2.0, false};   // Phosphorus (mg/kg)
KalmanState soilKFilter          = {0, 1.0, 0.05, 2.0, false};   // Potassium (mg/kg)

// Air sensors
KalmanState airHumidityFilter    = {0, 1.0, 0.05, 1.0, false};   // Humidity (%)
KalmanState airTempFilter        = {0, 1.0, 0.01, 0.5, false};   // Air temperature (°C)
KalmanState luxFilter            = {0, 1.0, 0.1, 10.0, false};  // Lux (light intensity)

// Đề xuất bón phân
float urea_kg_ha;    // kg/ha
float urea_g_plant;  // g/cây
float dap_kg_ha;
float dap_g_plant;  
float kcl_kg_ha;
float kcl_g_plant;
String recommendation_text;
bool needs_fertilization;

extern const NutrientThreshold tomatoThreshold;

float kalmanUpdate(KalmanState &state, float measurement) {
  if (!state.initialized) {
    state.x_est = measurement;
    state.P = 1.0f;
    state.initialized = true;
    return state.x_est;
  }

  float K = state.P / (state.P + state.R);
  state.x_est = state.x_est + K * (measurement - state.x_est);
  state.P = (1 - K) * state.P + state.Q;

  return state.x_est;
}

// Tính áp suất khí quyển (kPa) dựa trên độ cao
float calculate_P() {
    return P0 * pow((TEMP_REF - LAPSE_RATE * z) / TEMP_REF, GRAVITY_EXP);
}

// Tính psychrometric constant (kPa/°C)
float calculate_gamma() {
    return GAMMA_CONST * calculate_P();
}

// Tính latent heat of vaporization (MJ/kg)
float calculate_lambda(float T) {
    return LAMBDA_BASE - LAMBDA_SLOPE * T;
}

// Tính slope of saturation vapor pressure curve (kPa/°C)
float calculate_delta(float T) {
    float es = ES_CONST * exp(ES_SLOPE * T / (T + ES_TEMP_OFFSET));
    return DELTA_CONST * es / pow(T + ES_TEMP_OFFSET, 2);
}

// Tính ET0 bằng Priestley-Taylor (mm/day)
float computeET0() {
    float gamma = calculate_gamma();
    float lambda = calculate_lambda(temperature);
    
    // Áp suất bão hòa (es) và áp suất hơi nước thực tế (ea)
    float es = ES_CONST * exp(ES_SLOPE * temperature / (temperature + ES_TEMP_OFFSET));
    float ea = es * humidity / 100.0;  // Sử dụng humidity (%)
    
    float delta = calculate_delta(temperature);
    
    // Bức xạ ròng (MJ/m²/day, từ lux)
    float Rn = visibleLux * LUX_TO_RN * GREENHOUSE_FACTOR;
    
    // ET0 (mm/day)
    float ET0 = ALPHA * (delta / (delta + gamma)) * (Rn - G) / lambda;
    
    if (ET0 < 0) return ET0 = 0;
    return ET0;
}

// Cập nhật độ ẩm đất và tính lượng nước tưới
void updateSoilMoisture(bool isScheduledIrrigation) {
    // Tính TAW và RAW
    float TAW = (theta_fc - theta_wp) * Zr * 1000.0;  // mm
    float RAW = p * TAW;                              // mm
    
    // Khởi tạo SM lần đầu từ soilMoisture
    static bool firstRun = true;
    if (firstRun && soilMoisture >= theta_wp && soilMoisture <= theta_fc) {
        SM = (soilMoisture - theta_wp) / (theta_fc - theta_wp) * TAW;
        firstRun = false;
    } else if (firstRun) {
        SM = TAW * 0.5;  // Mặc định 50% TAW
        firstRun = false;
    }
    
    // Lưu SM_prev
    SM_prev = SM;
    
    // Tính ETc
    float ETc = Kc * computeET0();
    
    // Tính I_needed trước khi cập nhật SM
    if (SM_prev < RAW) {
        I_needed = RAW - SM_prev;
    } else {
        I_needed = 0;
    }

    if (!isScheduledIrrigation && (SM_prev >= RAW_THRESHOLD * RAW || I_needed < MIN_I_NEEDED)) {
        I_needed = 0;
    }
    
    // Cập nhật SM
    SM = SM_prev + I_needed - ETc - D;
    
    // Giới hạn SM
    SM = constrain(SM, 0, TAW);
    
    // Hiệu chỉnh SM dựa trên soilMoisture
    if (soilMoisture >= theta_wp && soilMoisture <= theta_fc) {
        float SM_sensor = (soilMoisture - theta_wp) / (theta_fc - theta_wp) * TAW;
        SM = SM_MODEL_WEIGHT * SM + SM_SENSOR_WEIGHT * SM_sensor;
    }
    
    // In debug khi tưới
    Serial.printf("ETc: %.2f mm/day | SM_prev: %.2f mm\n", ETc, SM_prev);
    Serial.printf("TAW: %.2f mm | RAW: %.2f mm\n", TAW, RAW);
    Serial.printf("SM: %.2f mm | I_irrigation: %.2f mm\n", SM, I_needed);
    Serial.printf("Irrigation: %s\n\n", isScheduledIrrigation ? "Scheduled" : "Supplemental");
}

StageFactor getStageFactor(int growthStage) {
  switch(growthStage) {
    case 1: // ƯƠM MẦM (0-3 tuần) - Ưu tiên P
      return {0.8f, 1.0f, 0.6f, "Ưu tiên P cho rễ"};
      
    case 2: // SINH TRƯỞNG (3-6 tuần) - Ưu tiên N  
      return {1.0f, 0.8f, 0.8f, "Ưu tiên N cho thân lá"};
      
    case 3: // RA HOA (6-9 tuần) - Ưu tiên P + K
      return {0.6f, 1.0f, 1.0f, "Ưu tiên P + K cho hoa quả"};
      
    case 4: // NUÔI QUẢ (9+ tuần) - Ưu tiên K
      return {0.6f, 0.8f, 1.2f, "Ưu tiên K cho chất lượng quả"};
      
    default:
      return {1.0f, 1.0f, 1.0f, "Giai đoạn không xác định"};
  }
}

// HÀM TẠO THÔNG BÁO
String generateRecommendationText(bool need_N, bool need_P, bool need_K,
                                 bool excess_N, bool excess_P, bool excess_K,
                                 float urea_g, float dap_g, float kcl_g,
                                 int stage, const char* priority) {
  String result = "=== ĐỀ XUẤT BÓN PHÂN - GĐ " + String(stage) + " ===\n";
  result += "📊 Ngưỡng: N(" + String(tomatoThreshold.N_low, 1) + "-" + String(tomatoThreshold.N_high, 1) + "%) ";
  result += "P(" + String(tomatoThreshold.P_low, 2) + "-" + String(tomatoThreshold.P_high, 2) + "%) ";
  result += "K(" + String(tomatoThreshold.K_low, 1) + "-" + String(tomatoThreshold.K_high, 1) + "%)\n";
  result += "🎯 " + String(priority) + "\n\n";
  
  // ĐỀ XUẤT CỤ THỂ
  if (urea_g > 0) {
    result += "• Urea: " + String(urea_g, 1) + " g/cây (" + String(urea_g * 25, 0) + " kg/ha)\n";
  }
  if (dap_g > 0) {
    result += "• DAP: " + String(dap_g, 1) + " g/cây (" + String(dap_g * 25, 0) + " kg/ha)\n";
  }
  if (kcl_g > 0) {
    result += "• KCl: " + String(kcl_g, 1) + " g/cây (" + String(kcl_g * 25, 0) + " kg/ha)\n";
  }
  
  // CẢNH BÁO THỪA DINH DƯỠNG
  if (excess_N || excess_P || excess_K) {
    result += "\n⚠️ CẢNH BÁO THỪA: ";
    if (excess_N) result += "N ";
    if (excess_P) result += "P ";
    if (excess_K) result += "K ";
    result += "- NGỪNG BÓN\n";
  }
  
  // TRƯỜNG HỢP ĐỦ DINH DƯỠNG
  if (!need_N && !need_P && !need_K && !excess_N && !excess_P && !excess_K) {
    result += "✅ ĐỦ DINH DƯỠNG - Không cần bón phân\n";
  }
  
  // KHUYẾN NGHỊ BỔ SUNG
  result += "\n💡 KHUYẾN NGHỊ: ";
  if (stage == 1 && need_P) result += "Chia nhỏ liều DAP, tưới gốc";
  else if (stage == 2 && need_N) result += "Chia 2 lần bón, cách nhau 7 ngày";
  else if (stage == 3 && (need_P || need_K)) result += "Bón trước khi ra hoa rộ";
  else if (stage == 4 && need_K) result += "Bón KCl để tăng chất lượng quả";
  else result += "Theo dõi định kỳ 7 ngày/lần";
  
  return result;
}

void analyzeNutrientNeeds() {
  // Lấy hệ số giai đoạn sinh trưởng
  StageFactor factor = getStageFactor(growthStage);

  // Phân tích trạng thái dinh dưỡng
  bool need_N = (soilN < tomatoThreshold.N_low);
  bool need_P = (soilP < tomatoThreshold.P_low);
  bool need_K = (soilK < tomatoThreshold.K_low);
  
  bool excess_N = (soilN > tomatoThreshold.N_high);
  bool excess_P = (soilP > tomatoThreshold.P_high);
  bool excess_K = (soilK > tomatoThreshold.K_high);

  // LIỀU LƯỢNG CƠ BẢN (kg/ha)
  float base_urea = 0, base_dap = 0, base_kcl = 0;
  
  // Tính toán liều lượng dựa trên nhu cầu
  if (need_N && !excess_N) {
    base_urea = 100.0f; // kg/ha cơ bản
  }
  if (need_P && !excess_P) {
    base_dap = 50.0f;   // kg/ha cơ bản  
  }
  if (need_K && !excess_K) {
    base_kcl = 80.0f;   // kg/ha cơ bản
  }

   // ÁP DỤNG HỆ SỐ GIAI ĐOẠN
  urea_kg_ha = base_urea * factor.urea_factor;
  dap_kg_ha = base_dap * factor.dap_factor;
  kcl_kg_ha = base_kcl * factor.kcl_factor;

  // QUY ĐỔI SANG G/CÂY (mật độ 25,000 cây/ha)
  urea_g_plant = urea_kg_ha * 1000 / 25000;
  dap_g_plant = dap_kg_ha * 1000 / 25000;
  kcl_g_plant = kcl_kg_ha * 1000 / 25000;

   // LÀM TRÒN ĐẾN 1 SỐ THẬP PHÂN
  urea_g_plant = round(urea_g_plant * 10) / 10;
  dap_g_plant = round(dap_g_plant * 10) / 10; 
  kcl_g_plant = round(kcl_g_plant * 10) / 10;

  recommendation_text = generateRecommendationText(
    need_N, need_P, need_K, excess_N, excess_P, excess_K,
    urea_g_plant, dap_g_plant, kcl_g_plant,
    growthStage, factor.priority_note
  );

  needs_fertilization = (base_urea > 0 || base_dap > 0 || base_kcl > 0);
}