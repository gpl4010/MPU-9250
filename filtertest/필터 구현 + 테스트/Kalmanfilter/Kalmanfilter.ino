#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;

class KalmanFilter {
private:
    float Q_angle, Q_bias;    // 프로세스 노이즈
    float R_measure;          // 측정 노이즈
    float angle, bias;        // 상태 변수
    float P[2][2];           // 오차 공분산 행렬
    
public:
    KalmanFilter() {
        Q_angle = 0.001f;    // 프로세스 노이즈의 분산
        Q_bias = 0.003f;
        R_measure = 0.03f;   // 측정 노이즈의 분산
        
        angle = 0.0f;
        bias = 0.0f;
        
        P[0][0] = 0.0f;
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 0.0f;
    }
    
    float update(float newAngle, float newRate, float dt) {
        // 예측
        angle += dt * (newRate - bias);
        
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        
        // 보정작업
        float y = newAngle - angle;
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        
        angle += K[0] * y;
        bias += K[1] * y;
        
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
        
        return angle;
    }
    
    void setAngle(float angle) {
        this->angle = angle;
    }
    
    void setQangle(float Q_angle) {
        this->Q_angle = Q_angle;
    }
    
    void setQbias(float Q_bias) {
        this->Q_bias = Q_bias;
    }
    
    void setRmeasure(float R_measure) {
        this->R_measure = R_measure;
    }
};

// 각 축에 대한 칼만 필터 인스턴스 생성
KalmanFilter kalmanX;
KalmanFilter kalmanY;
KalmanFilter kalmanZ;

float roll = 0, pitch = 0, yaw = 0;
unsigned long prevTime = 0;

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
    
    // 초기 각도 계산
    mpu.update();
    float accX = mpu.getAccX();
    float accY = mpu.getAccY();
    float accZ = mpu.getAccZ();
    
    // 초기 Roll과 Pitch 계산
    roll = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    
    // 칼만 필터 초기화
    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);
    kalmanZ.setAngle(yaw);
    
    prevTime = millis();
    Serial.println("센서 설정이 완료되었습니다!");
}

void loop() {
    if (mpu.update()) {
        unsigned long currentTime = millis();
        float dt = (currentTime - prevTime) / 1000.0f;
        
        // 센서 데이터 읽기
        float accX = mpu.getAccX();
        float accY = mpu.getAccY();
        float accZ = mpu.getAccZ();
        float gyroX = mpu.getGyroX();
        float gyroY = mpu.getGyroY();
        float gyroZ = mpu.getGyroZ();
        float magX = mpu.getMagX();
        float magY = mpu.getMagY();
        float magZ = mpu.getMagZ();
        
        // Roll,Pitch 계산
        float accRoll = atan2(accY, accZ) * RAD_TO_DEG;
        float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        
        //Yaw 계산
        float magYaw = atan2(magY, magX) * RAD_TO_DEG;
        
        // 칼만 필터 적용
        roll = kalmanX.update(accRoll, gyroX, dt);
        pitch = kalmanY.update(accPitch, gyroY, dt);
        yaw = kalmanZ.update(magYaw, gyroZ, dt);
        

        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 40) {
            Serial.print(roll, 2);
            Serial.print(",");
            Serial.print(pitch, 2);
            Serial.print(",");
            Serial.println(yaw, 2);
            prev_ms = millis();
        }
        
        prevTime = currentTime;
    }
}