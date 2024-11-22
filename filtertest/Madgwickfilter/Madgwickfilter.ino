#include <MPU9250.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

MPU9250 mpu;
Madgwick filter;

// 센서 샘플링 주파수 설정
const float sampleFreq = 100.0f;  // 100 Hz

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // MPU9250 초기화
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU9250 연결 실패!");
            delay(5000);
        }
    }

    // 센서 캘리브레이션
    Serial.println("센서 캘리브레이션을 시작합니다...");
    delay(2000);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    
    // Madgwick 필터 초기화
    filter.begin(sampleFreq);
    
    Serial.println("센서 설정이 완료되었습니다!");
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + (1000 / sampleFreq)) {  // sampleFreq Hz로 데이터 처리
            // 센서에서 원시 데이터 읽기
            float gx = mpu.getGyroX() * DEG_TO_RAD; // 각속도를 rad/s로 변환
            float gy = mpu.getGyroY() * DEG_TO_RAD;
            float gz = mpu.getGyroZ() * DEG_TO_RAD;
            float ax = mpu.getAccX();
            float ay = mpu.getAccY();
            float az = mpu.getAccZ();
            float mx = mpu.getMagX();
            float my = mpu.getMagY();
            float mz = mpu.getMagZ();

            // 자기장 데이터 정규화
            float mag_norm = sqrt(mx*mx + my*my + mz*mz);
            if (mag_norm > 0) {
                mx /= mag_norm;
                my /= mag_norm;
                mz /= mag_norm;
            }

            // Madgwick 필터 업데이트 (9축 모두 사용)
            filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

            // 각도 값 출력
            float roll = filter.getRoll();
            float pitch = filter.getPitch();
            float yaw = filter.getYaw();
            
            // 결과 출력
            Serial.print("Roll : ");
            Serial.print(roll, 2);
            Serial.print("  Pitch : ");
            Serial.print(pitch, 2);
            Serial.print("  Yaw : ");
            Serial.println(yaw, 2);
            
            prev_ms = millis();
        }
    }
}