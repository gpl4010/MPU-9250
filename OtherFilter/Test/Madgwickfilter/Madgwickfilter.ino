#include <MPU9250.h>
#include <MadgwickAHRS.h>

MPU9250 mpu;
Madgwick filter;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.setup(0x68)) {
        while(1) {
            Serial.println("MPU connection failed");
            delay(1000);
        }
    }
    
    // 기본 캘리브레이션
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    
    // 필터 초기화
    filter.begin(100.0f);  // 100Hz
}

void loop() {
    if (mpu.update()) {
        // Raw 데이터 읽기
        float gx = mpu.getGyroX() * DEG_TO_RAD;
        float gy = mpu.getGyroY() * DEG_TO_RAD;
        float gz = mpu.getGyroZ() * DEG_TO_RAD;
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();        
        float mx = mpu.getMagX();
        float my = mpu.getMagY();
        float mz = mpu.getMagZ();
        
        // 필터 업데이트
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        
        // 각도 출력
        Serial.print(filter.getRoll());
        Serial.print(",");
        Serial.print(filter.getPitch());
        Serial.print(",");
        Serial.println(filter.getYaw());
    }
}