#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // MPU9250 초기화
    if (!mpu.setup(0x68)) {  // I2C 주소는 일반적으로 0x68입니다
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
    
    Serial.println("센서 설정이 완료되었습니다!");
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {  // 40Hz로 데이터 출력
            float roll = mpu.getRoll();
            float pitch = mpu.getPitch();
            float yaw = mpu.getYaw();
            
            Serial.print(roll);
            Serial.print(",");
            Serial.print(pitch);
            Serial.print(",");
            Serial.println(yaw);
            
            prev_ms = millis();
        }
    }
}