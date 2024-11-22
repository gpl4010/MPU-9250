#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;

class ExtendedKalmanFilter {
private:
    // 상태 벡터: [q0, q1, q2, q3, bx, by, bz]
    float q0, q1, q2, q3;  // 쿼터니언
    float bx, by, bz;      // 자이로 바이어스
    float P[7][7];         // 오차 공분산
    const float dt;
    const float gyroNoise;
    const float accNoise;
    const float magNoise;
    
public:
    ExtendedKalmanFilter() : 
        dt(0.01f),          // 100Hz 샘플링
        gyroNoise(0.001f),  // 자이로스코프 노이즈
        accNoise(0.01f),    // 가속도계 노이즈
        magNoise(0.01f)     // 자기장 센서 노이즈
    {
        // 상태 초기화
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;  // 초기 쿼터니언
        bx = 0.0f; by = 0.0f; bz = 0.0f;              // 초기 바이어스
        
        // 공분산 행렬 초기화
        for(int i = 0; i < 7; i++) {
            for(int j = 0; j < 7; j++) {
                P[i][j] = (i == j) ? 0.01f : 0.0f;
            }
        }
    }
    
    void normalizeQuaternion() {
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        // 자이로 바이어스 보정
        float wx = gx - bx;
        float wy = gy - by;
        float wz = gz - bz;
        
        // 쿼터니언 업데이트 (간단한 적분)
        float dq0 = 0.5f * (-q1*wx - q2*wy - q3*wz);
        float dq1 = 0.5f * (q0*wx + q2*wz - q3*wy);
        float dq2 = 0.5f * (q0*wy - q1*wz + q3*wx);
        float dq3 = 0.5f * (q0*wz + q1*wy - q2*wx);
        
        q0 += dq0 * dt;
        q1 += dq1 * dt;
        q2 += dq2 * dt;
        q3 += dq3 * dt;
        
        normalizeQuaternion();
        
        // 가속도계 업데이트
        float norm_acc = sqrt(ax*ax + ay*ay + az*az);
        if (norm_acc > 0.01f) {
            ax /= norm_acc;
            ay /= norm_acc;
            az /= norm_acc;
            
            // 예측된 중력 방향
            float gx_pred = 2*(q1*q3 - q0*q2);
            float gy_pred = 2*(q2*q3 + q0*q1);
            float gz_pred = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            
            // 간단한 보정
            float error_x = ay*gz_pred - az*gy_pred;
            float error_y = az*gx_pred - ax*gz_pred;
            float error_z = ax*gy_pred - ay*gx_pred;
            
            // 쿼터니언 보정
            float correction = 0.01f;  // 보정 게인
            q0 += correction * (-q1*error_x - q2*error_y - q3*error_z);
            q1 += correction * (q0*error_x + q2*error_z - q3*error_y);
            q2 += correction * (q0*error_y - q1*error_z + q3*error_x);
            q3 += correction * (q0*error_z + q1*error_y - q2*error_x);
            
            normalizeQuaternion();
        }
        
        // 자이로 바이어스 업데이트
        float bias_correction = 0.001f;  // 바이어스 보정 게인
        bx += bias_correction * wx;
        by += bias_correction * wy;
        bz += bias_correction * wz;
    }
    
    void getEulerAngles(float& roll, float& pitch, float& yaw) {
        // Roll (x-axis rotation)
        roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
        
        // Pitch (y-axis rotation)
        float sinp = 2*(q0*q2 - q3*q1);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI/2, sinp);
        else
            pitch = asin(sinp);
        
        // Yaw (z-axis rotation)
        yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
        
        // 라디안에서 도(degree)로 변환
        roll *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;
    }
};

ExtendedKalmanFilter ekf;
unsigned long prevTime = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU9250 연결 실패!");
            delay(5000);
        }
    }

    Serial.println("센서 캘리브레이션을 시작합니다...");
    delay(2000);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    
    prevTime = micros();
    Serial.println("센서 설정이 완료되었습니다!");
}

void loop() {
    if (mpu.update()) {
        unsigned long currentTime = micros();
        float dt = (currentTime - prevTime) / 1000000.0f;
        
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
        
        // EKF 업데이트
        ekf.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        
        // 결과 출력 (25ms 마다)
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            float roll, pitch, yaw;
            ekf.getEulerAngles(roll, pitch, yaw);
            
            Serial.print("Roll : ");
            Serial.print(roll, 2);
            Serial.print("  Pitch : ");
            Serial.print(pitch, 2);
            Serial.print("  Yaw : ");
            Serial.println(yaw, 2);
            prev_ms = millis();
        }
        
        prevTime = currentTime;
    }
}