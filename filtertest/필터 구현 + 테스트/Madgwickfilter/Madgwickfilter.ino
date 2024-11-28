#include <MPU9250.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

MPU9250 mpu;
Madgwick filter;

const float sampleFreqDef = 512.0f;     // 샘플링 주파수
const int OUTPUT_RATE = 40;             // 출력 주기 (Hz)
const int OUTPUT_PERIOD = 1000/OUTPUT_RATE;

// 자기장 캘리브레이션 변수
float mag_bias[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU9250 연결 실패!");
            delay(5000);
        }
    }

    // 센서 캘리브레이션
    Serial.println("캘리브레이션을 위해 센서를 움직이지 마세요...");
    delay(2000);
    calibrateSensors();
    
    // Madgwick 필터 초기화
    filter.begin(sampleFreqDef);

    Serial.println("센서 초기화 완료!");
}

void calibrateSensors() {
    // 가속도계와 자이로스코프 캘리브레이션
    mpu.calibrateAccelGyro();
    
    // 자기장 센서 상세 캘리브레이션
    Serial.println("Rotate the device in figure-8 pattern for magnetometer calibration...");
    delay(3000);
    mpu.calibrateMag();
    
    // 자기장 바이어스 값 저장
    mag_bias[0] = mpu.getMagBiasX();
    mag_bias[1] = mpu.getMagBiasY();
    mag_bias[2] = mpu.getMagBiasZ();
}

void processMagData(float* mx, float* my, float* mz) {
    // 바이어스 보정
    *mx = (*mx - mag_bias[0]) * mag_scale[0];
    *my = (*my - mag_bias[1]) * mag_scale[1];
    *mz = (*mz - mag_bias[2]) * mag_scale[2];

    // 정규화
    float mag_norm = sqrt((*mx)*(*mx) + (*my)*(*my) + (*mz)*(*mz));
    if (mag_norm > 0) {
        *mx /= mag_norm;
        *my /= mag_norm;
        *mz /= mag_norm;
    }
}

void loop() {
    static uint32_t prev_process_us = micros();
    static uint32_t prev_output_ms = millis();
    
    if (mpu.update()) {
        uint32_t now_us = micros();
        float dt = (now_us - prev_process_us) / 1000000.0f;
        
        if (dt >= 1.0f/sampleFreqDef) {
            // 센서 데이터 읽기
            float gx = mpu.getGyroX() * DEG_TO_RAD;
            float gy = mpu.getGyroY() * DEG_TO_RAD;
            float gz = mpu.getGyroZ() * DEG_TO_RAD;
            
            float ax = mpu.getAccX();
            float ay = mpu.getAccY();
            float az = mpu.getAccZ();
            
            float mx = mpu.getMagX();
            float my = mpu.getMagY();
            float mz = mpu.getMagZ();
            
            // 자기장 데이터 처리
            processMagData(&mx, &my, &mz);
            
            // Madgwick 필터 업데이트
            filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
            
            prev_process_us = now_us;
            
            if (millis() - prev_output_ms >= OUTPUT_PERIOD) {
                float roll = filter.getRoll();
                float pitch = filter.getPitch();
                float yaw = filter.getYaw();
                
                // 각도 범위 정규화
                if (yaw < 0) yaw += 360.0f;
                
                Serial.print(roll, 2);
                Serial.print(",");
                Serial.print(pitch, 2);
                Serial.print(",");
                Serial.println(yaw, 2);
                
                prev_output_ms = millis();
            }
        }
    }
}