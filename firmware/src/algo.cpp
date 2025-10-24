#include "algo.h"
#include "sensor.h"
#include <Arduino.h>

// Biến toàn cục từ cảm biến
extern float temperature;      // °C
extern float humidity;         // % (độ ẩm tương đối)
extern float visibleLux;    // lux
extern float soilMoisture;     // fraction (0-1)
extern float soilTemperature;  // °C
extern int soilN;
extern int soilP;
extern int soilK;
extern uint8_t growthStage;

// Tham số cho cà chua trong nhà kính
float Kc = 0.8;               // Hệ số cây trồng (0.6-1.2 tùy giai đoạn)
float SM;                     // Độ ẩm đất hiện tại (mm)
float SM_prev;                // Độ ẩm đất trước đó (mm)
float D;               // Drainage (mm/day, tương đương 0.5 mm/h * 24)
float I_needed = 0;           // Lượng nước cần tưới (mm/day)
float theta_fc = 0.35;        // Field Capacity (fraction, đất sạch trộn sẵn)
float theta_wp = 0.15;        // Wilting Point (fraction)
float Zr = 0.3;               // Độ sâu rễ (m)
float p = 0.5;                // Management Allowed Depletion (phù hợp cho cà chua)
float z = 100;                // Độ cao ước lượng (mét)
float LF = 0;

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
    
    // // Áp suất bão hòa (es) và áp suất hơi nước thực tế (ea)
    // float es = ES_CONST * exp(ES_SLOPE * temperature / (temperature + ES_TEMP_OFFSET));
    // float ea = es * humidity / 100.0;  // Sử dụng humidity (%)
    
    float delta = calculate_delta(temperature);
    
    // Bức xạ ròng (MJ/m²/day, từ lux)
    float Rn = visibleLux * LUX_TO_RN * GREENHOUSE_FACTOR;
    
    // ET0 (mm/day)
    float ET0 = ALPHA * (delta / (delta + gamma)) * (Rn - G) / lambda;
    
    if (ET0 < 0) return ET0 = 0;
    return ET0;
}

float computeLF() {
    float EC_drainage = 5.0 * EC_IRRIGATION;  // Quy tắc 5:1
    return EC_TARGET / EC_drainage;
}

float computeDrainage(float SM_prev, float TAW) {
    if (SM_prev <= TAW * 0.8f) return 0.0;
    return (SM_prev - TAW * 0.8f) * DRAINAGE_FACTOR;
}

// Cập nhật độ ẩm đất và tính lượng nước tưới
void updateSoilMoisture(bool isScheduledIrrigation) {
    // Tính TAW và RAW
    float TAW = (theta_fc - theta_wp) * Zr * 1000.0;  // mm
    float RAW = p * TAW;                              // mm
    
    // Khởi tạo SM lần đầu từ soilMoisture (chuyển đổi % sang fraction)
    static bool firstRun = true;
    float soilMoistureFraction = soilMoisture * MOISTURE_TO_FRACTION;  // % → fraction
    if (firstRun && soilMoistureFraction >= theta_wp && soilMoistureFraction <= theta_fc) {
        SM = (soilMoistureFraction - theta_wp) / (theta_fc - theta_wp) * TAW;
        firstRun = false;
    } else if (firstRun) {
        SM = TAW * 0.5;  // Mặc định 50% TAW
        firstRun = false;
    }
    
    // Lưu SM_prev
    SM_prev = SM;
    D = computeDrainage(SM_prev, TAW);
    
    // Tính ETc cải tiến
    float ETc = computeETcImproved();

    // Xử lý EC cao (đất mặn)
    if (soilEC > EC_THRESHOLD) {
        LF = computeLF();
        handleHighEC(); // Hiển thị khuyến nghị xử lý
        Serial.println("Leaching mode activated!");
    }
    
    // Phân tích pH và EC với Decision Table + Fuzzy Logic
    SoilRecommendation recommendation = generateSoilRecommendation();
    Serial.println(recommendation.detailed_advice);
    
    // Lưu trữ dữ liệu NPK lịch sử
    storeNPKHistory(soilN, soilP, soilK);
    
    // Tính I_req cơ bản
    float theta_t = theta_wp + (SM_prev / (Zr * 1000.0));
    float I_req = ((theta_fc - theta_t) * Zr * 1000.0);

    // Áp dụng LF nếu cần xả mặn
    if (LF > 0.0) {
        float leachingWater = computeLeachingWater(I_req);
        I_req += leachingWater;  // Thêm nước xả mặn
        Serial.printf("Leaching: +%.2f mm (LF=%.2f)\n", leachingWater, LF);
    }

    // I_required = bù deficit + ETc + dự phòng D
    float I_required = max(0.0f, I_req + ETc + D);
    
    // Quyết định tưới thông minh
    bool canIrrigate = shouldIrrigate();
    bool needsIrrigation = isScheduledIrrigation || SM_prev <= (TAW - RAW) || I_required >= MIN_I_NEEDED;
    
    if (canIrrigate && needsIrrigation) {
        I_needed = I_required;
        Serial.println("✅ Quyết định tưới: Điều kiện phù hợp");
    } else {
        I_needed = 0;
        if (!canIrrigate) {
            Serial.println("❌ Không tưới: Điều kiện không phù hợp");
        } else {
            Serial.println("ℹ️ Không tưới: Chưa cần thiết");
        }
    }
    
    // Cập nhật SM
    SM = SM_prev + I_needed - ETc - D;
    
    // Giới hạn SM
    SM = constrain(SM, 0, TAW);
    
    // Hiệu chỉnh SM dựa trên soilMoisture (chuyển đổi % sang fraction)
    if (soilMoistureFraction >= theta_wp && soilMoistureFraction <= theta_fc) {
        float SM_sensor = (soilMoistureFraction - theta_wp) / (theta_fc - theta_wp) * TAW;
        SM = SM_MODEL_WEIGHT * SM + SM_SENSOR_WEIGHT * SM_sensor;
    }
    
    // In debug tối ưu
    Serial.println("=== SMART AGRICULTURE STATUS ===");
    Serial.printf("🌡️  Soil: %.1f°C | 💧 Moisture: %.1f%% | 📊 pH: %.2f | ⚡ EC: %.3f mS/cm\n", 
                  soilTemperature, soilMoisture, soilPH, soilEC);
    Serial.printf("🌱 NPK: N=%d P=%d K=%d mg/kg | 📈 Stage: %d | 💧 Saving: %s\n", 
                  soilN, soilP, soilK, growthStage, isWaterSavingMode() ? "ON" : "OFF");
    Serial.printf("💧 Water: ETc=%.1f mm/day | SM=%.1f mm | Irrigate: %.1f mm\n", 
                  ETc, SM, I_needed);
    Serial.printf("🎯 Decision: %s | Status: %s\n", 
                  isScheduledIrrigation ? "Scheduled" : "Smart", 
                  canIrrigate ? "✅ READY" : "❌ BLOCKED");
    Serial.println("================================\n");
}

// ========== CÁC HÀM MỚI CHO TƯỚI THÔNG MINH ==========

// Hàm kiểm tra điều kiện tưới thông minh
bool shouldIrrigate() {
    // Không tưới nếu độ ẩm không khí quá cao (có thể mưa)
    if (humidity > HIGH_HUMIDITY_THRESHOLD) {
        Serial.println("❌ Không tưới: Độ ẩm cao (>80%)");
        return false;
    }
    
    // Không tưới nếu nhiệt độ đất quá thấp (< 15°C)
    if (soilTemperature < LOW_SOIL_TEMP_THRESHOLD) {
        Serial.println("❌ Không tưới: Nhiệt độ đất thấp (<15°C)");
        return false;
    }
    
    // Không tưới nếu đất đã đủ ẩm (tránh úng nước) - soilMoisture là %
    if (soilMoisture > HIGH_SOIL_MOISTURE_THRESHOLD) {
        Serial.printf("❌ Không tưới: Đất đã đủ ẩm (>%.1f%%)\n", HIGH_SOIL_MOISTURE_THRESHOLD);
        return false;
    }
    
    return true;
}

// Hệ số giai đoạn sinh trưởng cho tưới nước
float getGrowthStageFactor(int growthStage) {
    switch(growthStage) {
        case 1: // ƯƠM MẦM - ít nước
            return 0.6f;
        case 2: // SINH TRƯỞNG - nhiều nước
            return 1.2f;
        case 3: // RA HOA - vừa phải
            return 1.0f;
        case 4: // NUÔI QUẢ - ít nước để tăng chất lượng
            return 0.8f;
        default:
            return 1.0f;
    }
}

// Kiểm tra chế độ tiết kiệm nước
bool isWaterSavingMode() {
    // Kích hoạt khi EC cao (đất mặn) hoặc thiếu nước
    return (soilEC > EC_THRESHOLD) || (SM < (theta_fc - theta_wp) * Zr * 1000.0 * 0.3);
}

// Hệ số tiết kiệm nước
float getWaterSavingFactor() {
    return isWaterSavingMode() ? WATER_SAVING_FACTOR : 1.0f;
}

// Tính ETc cải tiến
float computeETcImproved() {
    float baseETc = Kc * computeET0();
    
    // Điều chỉnh theo giai đoạn sinh trưởng
    float stageFactor = getGrowthStageFactor(growthStage);
    
    // Điều chỉnh theo nhiệt độ đất
    float tempFactor = 1.0 + (soilTemperature - 25.0) * 0.02;
    tempFactor = constrain(tempFactor, 0.5, 1.5); // Giới hạn 0.5-1.5
    
    // Áp dụng hệ số tiết kiệm nước
    float savingFactor = getWaterSavingFactor();
    
    float finalETc = baseETc * stageFactor * tempFactor * savingFactor;
    
    Serial.printf("ETc: Base=%.2f, Stage=%.2f, Temp=%.2f, Saving=%.2f, Final=%.2f\n", 
                  baseETc, stageFactor, tempFactor, savingFactor, finalETc);
    
    return max(0.0f, finalETc);
}

// ========== XỬ LÝ EC CAO (ĐẤT MẶN) ==========

// Tính tỷ lệ xả mặn cần thiết
float computeLeachingFraction() {
    if (soilEC <= EC_THRESHOLD) return 0.0f;
    
    // Công thức: LF = EC_target / (5 * EC_irrigation)
    // Với EC_target = 2.0, EC_irrigation = EC hiện tại
    float EC_irrigation = soilEC;
    float LF = EC_TARGET / (5.0 * EC_irrigation);
    
    // Giới hạn LF từ 0.1 đến 0.5 (10-50%)
    LF = constrain(LF, 0.1f, 0.5f);
    
    Serial.printf("Leaching: EC=%.2f, LF=%.2f (%.1f%%)\n", soilEC, LF, LF*100);
    return LF;
}

// Tính lượng nước cần để xả mặn
float computeLeachingWater(float baseIrrigation) {
    float LF = computeLeachingFraction();
    return baseIrrigation * LF;
}

// Kiểm tra EC nguy hiểm
bool isECCritical() {
    return soilEC > EC_CRITICAL_THRESHOLD;
}

// Kiểm tra EC khẩn cấp
bool isECEmergency() {
    return soilEC > EC_EMERGENCY_THRESHOLD;
}

// Lấy trạng thái EC
String getECStatus() {
    if (isECEmergency()) {
        return "🚨 KHẨN CẤP (EC > 4.0)";
    } else if (isECCritical()) {
        return "⚠️ NGUY HIỂM (EC > 3.0)";
    } else if (soilEC > EC_THRESHOLD) {
        return "⚠️ CAO (EC > 2.5)";
    } else {
        return "✅ BÌNH THƯỜNG";
    }
}

// Xử lý EC cao
void handleHighEC() {
    Serial.println("=== XỬ LÝ EC CAO ===");
    Serial.println(getECStatus());
    
    if (isECEmergency()) {
        Serial.println("🚨 KHUYẾN NGHỊ KHẨN CẤP:");
        Serial.println("1. Ngừng bón phân ngay lập tức");
        Serial.println("2. Tưới xả mặn với lượng nước gấp 2-3 lần bình thường");
        Serial.println("3. Cải thiện thoát nước");
        Serial.println("4. Sử dụng nước tưới chất lượng tốt");
    } else if (isECCritical()) {
        Serial.println("⚠️ KHUYẾN NGHỊ:");
        Serial.println("1. Giảm bón phân");
        Serial.println("2. Tăng tần suất tưới xả mặn");
        Serial.println("3. Kiểm tra nguồn nước tưới");
    } else if (soilEC > EC_THRESHOLD) {
        Serial.println("ℹ️ THEO DÕI:");
        Serial.println("1. Áp dụng chế độ tiết kiệm nước");
        Serial.println("2. Tưới xả mặn định kỳ");
        Serial.println("3. Kiểm tra EC thường xuyên");
    }
    Serial.println("==================\n");
}

// ========== DECISION TABLE + FUZZY LOGIC ==========

// Tính toán Fuzzy Membership cho pH
FuzzyMembership calculateFuzzyMembership(float ph) {
    FuzzyMembership membership = {0, 0, 0, 0, 0};
    
    // Very Acid (pH < 5.5)
    if (ph < PH_ACID_THRESHOLD) {
        membership.very_acid = 1.0f;
    } else if (ph < PH_ACID_THRESHOLD + FUZZY_OVERLAP) {
        membership.very_acid = (PH_ACID_THRESHOLD + FUZZY_OVERLAP - ph) / FUZZY_OVERLAP;
    }
    
    // Acid (pH 5.5-6.0)
    if (ph >= PH_ACID_THRESHOLD - FUZZY_OVERLAP && ph <= PH_OPTIMAL_LOW + FUZZY_OVERLAP) {
        if (ph <= PH_ACID_THRESHOLD) {
            membership.acid = (ph - (PH_ACID_THRESHOLD - FUZZY_OVERLAP)) / FUZZY_OVERLAP;
        } else if (ph <= PH_OPTIMAL_LOW) {
            membership.acid = 1.0f;
        } else {
            membership.acid = (PH_OPTIMAL_LOW + FUZZY_OVERLAP - ph) / FUZZY_OVERLAP;
        }
    }
    
    // Neutral (pH 6.0-6.5)
    if (ph >= PH_OPTIMAL_LOW - FUZZY_OVERLAP && ph <= PH_OPTIMAL_HIGH + FUZZY_OVERLAP) {
        if (ph <= PH_OPTIMAL_LOW) {
            membership.neutral = (ph - (PH_OPTIMAL_LOW - FUZZY_OVERLAP)) / FUZZY_OVERLAP;
        } else if (ph <= PH_OPTIMAL_HIGH) {
            membership.neutral = 1.0f;
        } else {
            membership.neutral = (PH_OPTIMAL_HIGH + FUZZY_OVERLAP - ph) / FUZZY_OVERLAP;
        }
    }
    
    // Alkaline (pH 6.5-7.5)
    if (ph >= PH_OPTIMAL_HIGH - FUZZY_OVERLAP && ph <= PH_ALKALINE_THRESHOLD + FUZZY_OVERLAP) {
        if (ph <= PH_OPTIMAL_HIGH) {
            membership.alkaline = (ph - (PH_OPTIMAL_HIGH - FUZZY_OVERLAP)) / FUZZY_OVERLAP;
        } else if (ph <= PH_ALKALINE_THRESHOLD) {
            membership.alkaline = 1.0f;
        } else {
            membership.alkaline = (PH_ALKALINE_THRESHOLD + FUZZY_OVERLAP - ph) / FUZZY_OVERLAP;
        }
    }
    
    // Very Alkaline (pH > 7.5)
    if (ph > PH_ALKALINE_THRESHOLD) {
        membership.very_alkaline = 1.0f;
    } else if (ph > PH_ALKALINE_THRESHOLD - FUZZY_OVERLAP) {
        membership.very_alkaline = (ph - (PH_ALKALINE_THRESHOLD - FUZZY_OVERLAP)) / FUZZY_OVERLAP;
    }
    
    return membership;
}

// Tìm rule phù hợp trong Decision Table
DecisionRule findMatchingRule(float ph, float ec) {
    // Decision Table Rules
    DecisionRule rules[] = {
        // pH < 5.5 (Very Acid) - EC đơn vị: mS/cm
        {0, 5.5, 0, EC_DECISION_LOW, "BÓN VÔI", "Bón CaCO₃ để tăng pH", 1},
        {0, 5.5, EC_DECISION_LOW, EC_DECISION_HIGH, "BÓN VÔI + RỬA MẶN", "Bón vôi + tưới xả mặn", 1},
        {0, 5.5, EC_DECISION_HIGH, 10.0, "KHẨN CẤP", "Ngừng bón phân + rửa mặn khẩn cấp", 1},
        
        // pH 5.5-6.0 (Acid)
        {5.5, 6.0, 0, EC_DECISION_LOW, "BÓN VÔI NHẸ", "Bón ít vôi để điều chỉnh pH", 2},
        {5.5, 6.0, EC_DECISION_LOW, EC_DECISION_HIGH, "BÓN VÔI + RỬA MẶN", "Bón vôi + tưới xả mặn", 1},
        {5.5, 6.0, EC_DECISION_HIGH, 10.0, "KHẨN CẤP", "Ngừng bón phân + rửa mặn khẩn cấp", 1},
        
        // pH 6.0-6.5 (Optimal)
        {6.0, 6.5, 0, EC_DECISION_LOW, "DUY TRÌ", "pH tối ưu, duy trì hiện tại", 3},
        {6.0, 6.5, EC_DECISION_LOW, EC_DECISION_HIGH, "RỬA MẶN", "Tưới xả mặn để giảm EC", 2},
        {6.0, 6.5, EC_DECISION_HIGH, 10.0, "KHẨN CẤP", "Ngừng bón phân + rửa mặn khẩn cấp", 1},
        
        // pH 6.5-7.5 (Alkaline)
        {6.5, 7.5, 0, EC_DECISION_LOW, "BÓN CHẤT HỮU CƠ", "Bón phân hữu cơ để giảm pH", 2},
        {6.5, 7.5, EC_DECISION_LOW, EC_DECISION_HIGH, "BÓN HỮU CƠ + RỬA MẶN", "Bón hữu cơ + tưới xả mặn", 1},
        {6.5, 7.5, EC_DECISION_HIGH, 10.0, "KHẨN CẤP", "Ngừng bón phân + rửa mặn khẩn cấp", 1},
        
        // pH > 7.5 (Very Alkaline)
        {7.5, 14.0, 0, EC_DECISION_LOW, "BÓN GYPSUM", "Bón gypsum để giảm pH", 1},
        {7.5, 14.0, EC_DECISION_LOW, EC_DECISION_HIGH, "BÓN GYPSUM + RỬA MẶN", "Bón gypsum + tưới xả mặn", 1},
        {7.5, 14.0, EC_DECISION_HIGH, 10.0, "KHẨN CẤP", "Ngừng bón phân + rửa mặn khẩn cấp", 1}
    };
    
    // Tìm rule phù hợp nhất
    DecisionRule bestRule = {0, 0, 0, 0, "KHÔNG XÁC ĐỊNH", "Không có khuyến nghị", 3};
    int bestPriority = 4;
    
    for (int i = 0; i < 15; i++) {
        if (ph >= rules[i].ph_min && ph < rules[i].ph_max &&
            ec >= rules[i].ec_min && ec < rules[i].ec_max &&
            rules[i].priority < bestPriority) {
            bestRule = rules[i];
            bestPriority = rules[i].priority;
        }
    }
    
    return bestRule;
}

// Tạo khuyến nghị tổng hợp
SoilRecommendation generateSoilRecommendation() {
    SoilRecommendation rec;
    
    // Phân tích pH
    FuzzyMembership phFuzzy = calculateFuzzyMembership(soilPH);
    if (phFuzzy.very_acid > FUZZY_STRONG) {
        rec.ph_status = "🚨 QUÁ CHUA (pH < 5.5)";
        rec.urgency_level = 1.0f;
    } else if (phFuzzy.acid > FUZZY_MODERATE) {
        rec.ph_status = "⚠️ HƠI CHUA (pH 5.5-6.0)";
        rec.urgency_level = 0.7f;
    } else if (phFuzzy.neutral > FUZZY_MODERATE) {
        rec.ph_status = "✅ TỐI ƯU (pH 6.0-6.5)";
        rec.urgency_level = 0.1f;
    } else if (phFuzzy.alkaline > FUZZY_MODERATE) {
        rec.ph_status = "⚠️ HƠI KIỀM (pH 6.5-7.5)";
        rec.urgency_level = 0.5f;
    } else {
        rec.ph_status = "🚨 QUÁ KIỀM (pH > 7.5)";
        rec.urgency_level = 1.0f;
    }
    
    // Phân tích EC
    if (soilEC > EC_EMERGENCY_THRESHOLD) {
        rec.ec_status = "🚨 EC KHẨN CẤP (>4.0)";
        rec.urgency_level = max(rec.urgency_level, 1.0f);
    } else if (soilEC > EC_CRITICAL_THRESHOLD) {
        rec.ec_status = "⚠️ EC NGUY HIỂM (>3.0)";
        rec.urgency_level = max(rec.urgency_level, 0.8f);
    } else if (soilEC > EC_THRESHOLD) {
        rec.ec_status = "⚠️ EC CAO (>2.5)";
        rec.urgency_level = max(rec.urgency_level, 0.6f);
    } else {
        rec.ec_status = "✅ EC BÌNH THƯỜNG";
    }
    
    // Tìm rule phù hợp
    DecisionRule rule = findMatchingRule(soilPH, soilEC);
    rec.primary_action = String(rule.action);
    rec.secondary_action = String(rule.recommendation);
    
    // Tạo lời khuyên chi tiết
    rec.detailed_advice = "=== KHUYẾN NGHỊ TỔNG HỢP ===\n";
    rec.detailed_advice += "pH: " + rec.ph_status + "\n";
    rec.detailed_advice += "EC: " + rec.ec_status + "\n";
    rec.detailed_advice += "Hành động chính: " + rec.primary_action + "\n";
    rec.detailed_advice += "Khuyến nghị: " + rec.secondary_action + "\n";
    rec.detailed_advice += "Mức độ khẩn cấp: " + String(rec.urgency_level * 100, 0) + "%\n";
    
    return rec;
}

// Lấy trạng thái pH
String getPHStatus() {
    FuzzyMembership phFuzzy = calculateFuzzyMembership(soilPH);
    
    if (phFuzzy.very_acid > FUZZY_STRONG) {
        return "🚨 QUÁ CHUA (pH < 5.5)";
    } else if (phFuzzy.acid > FUZZY_MODERATE) {
        return "⚠️ HƠI CHUA (pH 5.5-6.0)";
    } else if (phFuzzy.neutral > FUZZY_MODERATE) {
        return "✅ TỐI ƯU (pH 6.0-6.5)";
    } else if (phFuzzy.alkaline > FUZZY_MODERATE) {
        return "⚠️ HƠI KIỀM (pH 6.5-7.5)";
    } else {
        return "🚨 QUÁ KIỀM (pH > 7.5)";
    }
}

// Lấy trạng thái EC chi tiết
String getECStatusDetailed() {
    if (soilEC > EC_EMERGENCY_THRESHOLD) {
        return "🚨 KHẨN CẤP (EC > 4.0)";
    } else if (soilEC > EC_CRITICAL_THRESHOLD) {
        return "⚠️ NGUY HIỂM (EC > 3.0)";
    } else if (soilEC > EC_THRESHOLD) {
        return "⚠️ CAO (EC > 2.5)";
    } else {
        return "✅ BÌNH THƯỜNG (EC < 2.5)";
    }
}


// ========== HỆ THỐNG BÓN PHÂN THÔNG MINH ==========

// Lưu trữ dữ liệu NPK lịch sử
NPKData npkHistory[NPK_HISTORY_DAYS];
int npkHistoryIndex = 0;

void storeNPKHistory(float N, float P, float K) {
    npkHistory[npkHistoryIndex].N = N;
    npkHistory[npkHistoryIndex].P = P;
    npkHistory[npkHistoryIndex].K = K;
    npkHistory[npkHistoryIndex].timestamp = millis();
    npkHistory[npkHistoryIndex].valid = true;
    
    npkHistoryIndex = (npkHistoryIndex + 1) % NPK_HISTORY_DAYS;
    
    Serial.printf("NPK History stored: N=%.1f, P=%.1f, K=%.1f mg/kg\n", N, P, K);
}

// Lấy trung bình NPK 3 ngày gần nhất
NPKData getAverageNPK() {
    NPKData average = {0, 0, 0, 0, false};
    int validCount = 0;
    
    for (int i = 0; i < NPK_HISTORY_DAYS; i++) {
        if (npkHistory[i].valid) {
            average.N += npkHistory[i].N;
            average.P += npkHistory[i].P;
            average.K += npkHistory[i].K;
            validCount++;
        }
    }
    
    if (validCount > 0) {
        average.N /= validCount;
        average.P /= validCount;
        average.K /= validCount;
        average.valid = true;
    }
    
    return average;
}

// Lấy dữ liệu vườn mẫu năng suất cao
ReferenceGarden getReferenceGarden() {
    ReferenceGarden ref;
    
    // Dữ liệu vườn mẫu cà chua năng suất cao
    ref.N_optimal = 5.5f;      // mg/kg
    ref.P_optimal = 0.6f;      // mg/kg
    ref.K_optimal = 3.5f;      // mg/kg
    ref.yield_kg_ha = 80000.0f; // kg/ha
    ref.crop_type = "Cà chua";
    ref.season = "Mùa khô";
    
    return ref;
}

// Tính toán nhu cầu bón phân
FertilizerRecommendation calculateFertilizerNeeds() {
    FertilizerRecommendation rec;
    
    // Lấy dữ liệu trung bình NPK 3 ngày
    NPKData currentNPK = getAverageNPK();
    ReferenceGarden refGarden = getReferenceGarden();
    
    if (!currentNPK.valid) {
        rec.needs_fertilizer = false;
        rec.schedule_text = "Không đủ dữ liệu NPK lịch sử";
        return rec;
    }
    
    // Tính chênh lệch NPK
    rec.N_deficit = max(0.0f, refGarden.N_optimal - currentNPK.N);
    rec.P_deficit = max(0.0f, refGarden.P_optimal - currentNPK.P);
    rec.K_deficit = max(0.0f, refGarden.K_optimal - currentNPK.K);
    
    // Tính lượng phân cần bón (kg/ha)
    rec.urea_kg_ha = (rec.N_deficit * ABSORPTION_RATE_N) / FERTILIZER_EFFICIENCY * 2.0f;  // Urea chứa 46% N
    rec.dap_kg_ha = (rec.P_deficit * ABSORPTION_RATE_P) / FERTILIZER_EFFICIENCY * 1.5f;   // DAP chứa 18% N, 46% P
    rec.kcl_kg_ha = (rec.K_deficit * ABSORPTION_RATE_K) / FERTILIZER_EFFICIENCY * 1.2f;   // KCl chứa 60% K
    
    // Quy đổi sang g/cây
    rec.urea_g_plant = rec.urea_kg_ha * 1000.0f / PLANT_DENSITY;
    rec.dap_g_plant = rec.dap_kg_ha * 1000.0f / PLANT_DENSITY;
    rec.kcl_g_plant = rec.kcl_kg_ha * 1000.0f / PLANT_DENSITY;
    
    // Làm tròn
    rec.urea_g_plant = round(rec.urea_g_plant * 10) / 10;
    rec.dap_g_plant = round(rec.dap_g_plant * 10) / 10;
    rec.kcl_g_plant = round(rec.kcl_g_plant * 10) / 10;
    
    // Kiểm tra có cần bón phân không
    rec.needs_fertilizer = (rec.N_deficit > 0.5f || rec.P_deficit > 0.1f || rec.K_deficit > 0.5f);
    
    // Tạo lịch bón phân
    rec.schedule_text = "=== LỊCH BÓN PHÂN THÔNG MINH ===\n";
    rec.schedule_text += "Dựa trên so sánh với vườn mẫu năng suất cao\n";
    rec.schedule_text += "Chu kỳ bón: 2 tuần/lần\n\n";
    
    if (rec.needs_fertilizer) {
        rec.schedule_text += "KHUYẾN NGHỊ BÓN PHÂN:\n";
        if (rec.urea_g_plant > 0) {
            rec.schedule_text += "• Urea: " + String(rec.urea_g_plant, 1) + " g/cây (" + String(rec.urea_kg_ha, 0) + " kg/ha)\n";
        }
        if (rec.dap_g_plant > 0) {
            rec.schedule_text += "• DAP: " + String(rec.dap_g_plant, 1) + " g/cây (" + String(rec.dap_kg_ha, 0) + " kg/ha)\n";
        }
        if (rec.kcl_g_plant > 0) {
            rec.schedule_text += "• KCl: " + String(rec.kcl_g_plant, 1) + " g/cây (" + String(rec.kcl_kg_ha, 0) + " kg/ha)\n";
        }
    } else {
        rec.schedule_text += "✅ ĐỦ DINH DƯỠNG - Không cần bón phân\n";
    }
    
    return rec;
}

// Tạo lịch bón phân 6 tháng
FertilizerSchedule* generate6MonthSchedule() {
    static FertilizerSchedule schedule[12]; // 6 tháng x 2 tuần/tháng
    
    String months[] = {"Tháng 1", "Tháng 2", "Tháng 3", "Tháng 4", "Tháng 5", "Tháng 6"};
    String fertilizerTypes[] = {"Urea", "DAP", "KCl", "NPK tổng hợp"};
    
    for (int month = 0; month < 6; month++) {
        for (int week = 0; week < 2; week++) {
            int index = month * 2 + week;
            schedule[index].month = months[month];
            schedule[index].week = (week + 1) * 2; // Tuần 2, 4
            
            // Phân bón theo giai đoạn
            if (month < 2) { // Tháng 1-2: Ươm mầm
                schedule[index].fertilizer_type = "DAP";
                schedule[index].amount_kg_ha = 50.0f;
                schedule[index].amount_g_plant = 2.0f;
                schedule[index].application_method = "Tưới gốc";
                schedule[index].notes = "Ưu tiên P cho rễ";
            } else if (month < 4) { // Tháng 3-4: Sinh trưởng
                schedule[index].fertilizer_type = "Urea";
                schedule[index].amount_kg_ha = 100.0f;
                schedule[index].amount_g_plant = 4.0f;
                schedule[index].application_method = "Rải đều";
                schedule[index].notes = "Ưu tiên N cho thân lá";
            } else { // Tháng 5-6: Ra hoa - Nuôi quả
                schedule[index].fertilizer_type = "KCl";
                schedule[index].amount_kg_ha = 80.0f;
                schedule[index].amount_g_plant = 3.2f;
                schedule[index].application_method = "Tưới gốc";
                schedule[index].notes = "Ưu tiên K cho hoa quả";
            }
        }
    }
    
    return schedule;
}

// Định dạng lịch bón phân
String formatFertilizerSchedule() {
    FertilizerSchedule* schedule = generate6MonthSchedule();
    String result = "=== LỊCH BÓN PHÂN 6 THÁNG ===\n";
    result += "Chu kỳ: 2 tuần/lần\n";
    result += "Mật độ: 25,000 cây/ha\n\n";
    
    for (int i = 0; i < 12; i++) {
        result += schedule[i].month + " - Tuần " + String(schedule[i].week) + ":\n";
        result += "• Loại: " + schedule[i].fertilizer_type + "\n";
        result += "• Lượng: " + String(schedule[i].amount_kg_ha, 0) + " kg/ha (" + String(schedule[i].amount_g_plant, 1) + " g/cây)\n";
        result += "• Cách bón: " + schedule[i].application_method + "\n";
        result += "• Ghi chú: " + schedule[i].notes + "\n\n";
    }
    
    return result;
}

// In phân tích bón phân tối ưu
void printFertilizerAnalysis() {
    Serial.println("=== 🌱 FERTILIZER ANALYSIS ===");
    
    // Dữ liệu hiện tại
    NPKData currentNPK = getAverageNPK();
    ReferenceGarden refGarden = getReferenceGarden();
    
    Serial.printf("📊 Current NPK (3d): N=%.1f P=%.1f K=%.1f mg/kg\n", 
                  currentNPK.N, currentNPK.P, currentNPK.K);
    Serial.printf("🎯 Target NPK: N=%.1f P=%.1f K=%.1f mg/kg\n", 
                  refGarden.N_optimal, refGarden.P_optimal, refGarden.K_optimal);
    
    // Tính toán nhu cầu
    FertilizerRecommendation rec = calculateFertilizerNeeds();
    Serial.println(rec.schedule_text);
    
    // Lịch 6 tháng (chỉ hiển thị khi cần)
    // if (rec.needs_fertilizer) {
    //     Serial.println("📅 6-MONTH SCHEDULE:");
    //     Serial.println(formatFertilizerSchedule());
    // }
    Serial.println("============================\n");
}