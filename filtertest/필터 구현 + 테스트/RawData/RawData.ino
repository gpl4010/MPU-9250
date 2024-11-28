#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;

// 현재 각도 저장 변수
float roll = 0, pitch = 0, yaw = 0;
float accRoll = 0, accPitch=0,magYaw=0;
unsigned long prevTime = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU connection failed!");
            delay(5000);
        }
    }
}

void calculateAngles() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0f;  // 초단위 변환
    
    // 가속도계로부터 Roll과 Pitch 계산
    accRoll = atan2(mpu.getAccY(), mpu.getAccZ()) * RAD_TO_DEG;
    accPitch = atan2(-mpu.getAccX(), sqrt(mpu.getAccY() * mpu.getAccY() + mpu.getAccZ() * mpu.getAccZ())) * RAD_TO_DEG;

    // 자이로스코프 적분
    roll += mpu.getGyroX() * dt;
    pitch += mpu.getGyroY() * dt;
    yaw += mpu.getGyroZ() * dt;

    // 지자기 센서로부터 Yaw 계산
    magYaw = atan2(mpu.getMagY(), mpu.getMagX()) * RAD_TO_DEG;
    
    prevTime = currentTime;
}

void loop() {
    if (mpu.update()) {
        calculateAngles();

        //Serial.print(roll, 2);
        //Serial.print(",");
        //Serial.print(pitch, 2);
        //Serial.print(",");
        //Serial.println(yaw, 2);

        Serial.print(accRoll, 2);
        Serial.print(",");
        Serial.print(accPitch, 2);
        Serial.print(",");
        Serial.println(magYaw, 2);
    }
    delay(20); // 50Hz
}