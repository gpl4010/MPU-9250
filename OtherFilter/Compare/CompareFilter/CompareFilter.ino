#include <MPU9250.h>
#include <Wire.h>
#include "eigen.h"

using namespace Eigen;

MPU9250 mpu;

#define CALIBRATION_SWITCH 12
float offsetValues[9] = {0};

// 공통 상수
const float D2R = M_PI/180.0f;
const float R2D = 180.0f/M_PI;
const float GYRO_SCALE = (2000.0f / 32768.0f) * (PI / 180.0f);
const float ACC_SCALE = (4.0f / 32768.0f) * 9.81f;
const float MAG_SCALE = (4912.0f / 32760.0f);

// Raw Data 변수
float rawRoll = 0, rawPitch = 0, rawYaw = 0;
float accRoll = 0, accPitch = 0, magYaw = 0;
unsigned long prevTime = 0;

// 자이로 바이어스
float gyroBias[3] = {0, 0, 0};

class MyFilter {
private:
    float beta;
    float q0, q1, q2, q3;
    float sampleFreq;
    
public:
    MyFilter() { //쿼터니언 기반의 센서 융합 알고리즘 적용
        beta = 0.046f;
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        sampleFreq = 50.0f;  // 50Hz로 수정
    }
    
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
        float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // 가속도계 정규화
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 지자기 정규화
        recipNorm = 1.0f / sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 보조 변수
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // 쿼터니언 미분값 계산
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // 가속도계 보정
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // 정규화된 중력 벡터
            float _2q0 = 2.0f * q0;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _4q0 = 4.0f * q0;
            float _4q1 = 4.0f * q1;
            float _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1;
            float _8q2 = 8.0f * q2;

            // 기울기 계산
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

            // yaw값 보정을 위한 정규화
            recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // 쿼터니언 변화율 적용
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // 쿼터니언 적분
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    void getAngles(float* roll, float* pitch, float* yaw) {
        *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
        *pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
        *yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
    }
};


class ExtendedKalmanFilter {
private:
    Matrix<float, 6, 1> x_state;
    Matrix<float, 6, 6> P_cov;    
    Matrix<float, 6, 6> Q_noise;  
    Matrix<float, 2, 2> R_noise;
    float dt;

    Matrix<float, 2, 2> inverse2x2(const Matrix<float, 2, 2>& A) {
        Matrix<float, 2, 2> result;
        float det = A(0,0) * A(1,1) - A(0,1) * A(1,0);
        float invDet = 1.0f / det;
        
        result(0,0) = A(1,1) * invDet;
        result(0,1) = -A(0,1) * invDet;
        result(1,0) = -A(1,0) * invDet;
        result(1,1) = A(0,0) * invDet;
        
        return result;
    }

public:
    ExtendedKalmanFilter() : dt(0.01f) {
        x_state.setZero();
        P_cov.setIdentity();
        P_cov *= 0.1f;

        Q_noise.setIdentity();
        Q_noise.block<3,3>(0,0) *= 0.001f;
        Q_noise.block<3,3>(3,3) *= 0.003f;

        R_noise.setIdentity();
        R_noise *= 0.3f;
    }

    void predict(float gx, float gy, float gz) {
        float wx = gx - x_state(3);
        float wy = gy - x_state(4);
        float wz = gz - x_state(5);

        float sr = sin(x_state(0) * D2R);
        float cr = cos(x_state(0) * D2R);
        float tp = tan(x_state(1) * D2R);
        float cp = cos(x_state(1) * D2R);

        Matrix<float, 6, 1> x_pred;
        x_pred(0) = x_state(0) + dt * (wx + wy * sr * tp + wz * cr * tp) * R2D;
        x_pred(1) = x_state(1) + dt * (wy * cr - wz * sr) * R2D;
        x_pred(2) = x_state(2) + dt * (wy * sr / cp + wz * cr / cp) * R2D;
        x_pred(3) = x_state(3);
        x_pred(4) = x_state(4);
        x_pred(5) = x_state(5);

        Matrix<float, 6, 6> jacobianF;
        jacobianF.setIdentity();
        jacobianF(0,0) = 1 + dt * (wy * cr * tp - wz * sr * tp);
        jacobianF(0,1) = dt * (wy * sr + wz * cr) / (cp * cp);
        jacobianF(1,0) = dt * (-wy * sr - wz * cr);
        jacobianF(2,0) = dt * (wy * cr - wz * sr) / cp;
        jacobianF(2,1) = dt * (wy * sr + wz * cr) * tp / cp;

        P_cov = jacobianF * P_cov * jacobianF.transpose() + Q_noise;
        x_state = x_pred;
    }

    void update(float ax, float ay, float az) {
        float acc_norm = sqrt(ax*ax + ay*ay + az*az);
        if(acc_norm > 0.1f) {
            ax /= acc_norm;
            ay /= acc_norm;
            az /= acc_norm;

            float roll_m = atan2(ay, az) * R2D;
            float pitch_m = atan2(-ax, sqrt(ay*ay + az*az)) * R2D;

            Matrix<float, 2, 1> z;
            z << roll_m, pitch_m;

            Matrix<float, 2, 1> h;
            h << x_state(0), x_state(1);

            Matrix<float, 2, 6> H;
            H.setZero();
            H(0,0) = 1.0f;
            H(1,1) = 1.0f;

            Matrix<float, 2, 2> S = H * P_cov * H.transpose() + R_noise;
            Matrix<float, 6, 2> K = P_cov * H.transpose() * inverse2x2(S);

            Matrix<float, 2, 1> innovation = z - h;
            if(innovation(0) > 180.0f) innovation(0) -= 360.0f;
            else if(innovation(0) < -180.0f) innovation(0) += 360.0f;
            
            x_state += K * innovation;

            Matrix<float, 6, 6> I = Matrix<float, 6, 6>::Identity();
            P_cov = (I - K * H) * P_cov;
        }
    }

    void getEulerAngles(float& roll, float& pitch, float& yaw) {
        roll = x_state(0);
        pitch = x_state(1);
        yaw = x_state(2);
        
        while(yaw > 180.0f) yaw -= 360.0f;
        while(yaw < -180.0f) yaw += 360.0f;
    }

    void setDt(float new_dt) {
        dt = new_dt;
    }
};

MyFilter myFilter;
ExtendedKalmanFilter ekf;

void calibrateGyro() {
    float sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 1000;
    
    Serial.println("자이로스코프 초기화 중...");
    
    for(int i = 0; i < samples; i++) {
        if(mpu.update()) {
            sumX += mpu.getGyroX();
            sumY += mpu.getGyroY();
            sumZ += mpu.getGyroZ();
        }
        delay(1);
    }
    
    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;
    
    Serial.println("Gyro calibration complete!");
}

void calculateRawAngles() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0f;  // 초단위 변환

    // 가속도계로부터 Roll과 Pitch 계산
    accRoll = atan2(mpu.getAccY(), mpu.getAccZ()) * R2D;
    accPitch = atan2(-mpu.getAccX(), sqrt(mpu.getAccY() * mpu.getAccY() + mpu.getAccZ() * mpu.getAccZ())) * R2D;
    
    // 자이로스코프 센서로부터 Yaw 계산, 지자기센서로 계산시 0고정
    rawYaw += mpu.getGyroZ() * dt;

    prevTime = currentTime;
}

void performCalibration() {
    Serial.println("캘리브레이션 시작...");
    
    if(mpu.update()) {
        // MyFilter 현재값 저장
        float roll1, pitch1, yaw1;
        myFilter.getAngles(&roll1, &pitch1, &yaw1);
        offsetValues[0] = roll1;
        offsetValues[1] = pitch1;
        offsetValues[2] = yaw1;

        // Raw 데이터 현재값 저장
        calculateRawAngles();
        offsetValues[3] = accRoll;
        offsetValues[4] = accPitch;
        offsetValues[5] = rawYaw;

        // EKF 현재값 저장
        float roll3, pitch3, yaw3;
        ekf.getEulerAngles(roll3, pitch3, yaw3);
        offsetValues[6] = roll3;
        offsetValues[7] = pitch3;
        offsetValues[8] = yaw3;
    }

    Serial.println("캘리브레이션 완료!");
    delay(50);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    
    pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);

    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU9250 연결 실패!");
            delay(5000);
        }
    }

    delay(2000);
    Serial.println("센서 캘리브레이션 중...");

    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    calibrateGyro();

    Serial.println("센서 초기화 완료!");
}

void loop() {
    static unsigned long prev_ms = millis();
    
    if (mpu.update()) {
        unsigned long current_ms = millis();
        float dt = (current_ms - prev_ms) / 1000.0f;

        if(digitalRead(CALIBRATION_SWITCH) == HIGH) {
        performCalibration();
        }
        
        if (current_ms - prev_ms >= 20) {
            // 센서 데이터 읽기
            float gx = (mpu.getGyroX() - gyroBias[0]) * D2R;
            float gy = (mpu.getGyroY() - gyroBias[1]) * D2R;
            float gz = (mpu.getGyroZ() - gyroBias[2]) * D2R;
            float ax = mpu.getAccX();
            float ay = mpu.getAccY();
            float az = mpu.getAccZ();
            float mx = mpu.getMagX();
            float my = mpu.getMagY();
            float mz = mpu.getMagZ();

            // Raw 데이터 계산
            calculateRawAngles();

            // MyFilter 업데이트
            myFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
            float myRoll, myPitch, myYaw;
            myFilter.getAngles(&myRoll, &myPitch, &myYaw);

            // EKF 업데이트
            ekf.setDt(dt);
            ekf.predict(gx, gy, gz);
            ekf.update(ax, ay, az);
            float ekfRoll, ekfPitch, ekfYaw;
            ekf.getEulerAngles(ekfRoll, ekfPitch, ekfYaw);

            Serial.print(myRoll - offsetValues[0], 2); Serial.print(",");
            Serial.print(myPitch - offsetValues[1], 2); Serial.print(",");
            Serial.print(myYaw - offsetValues[2], 2); Serial.print(",");
            
            Serial.print(accRoll - offsetValues[3], 2); Serial.print(",");
            Serial.print(accPitch - offsetValues[4], 2); Serial.print(",");
            Serial.print(rawYaw - offsetValues[5], 2); Serial.print(",");
            
            Serial.print(ekfRoll - offsetValues[6], 2); Serial.print(",");
            Serial.print(ekfPitch - offsetValues[7], 2); Serial.print(",");
            Serial.println(ekfYaw - offsetValues[8], 2);
            
            prev_ms = current_ms;
        }
    }
}