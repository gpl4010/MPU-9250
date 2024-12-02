#include <MPU9250.h>
#include <Wire.h>
#include "eigen.h"

using namespace Eigen;

const float D2R = PI/180.0f;
const float R2D = 180.0f/PI;

MPU9250 mpu;

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

ExtendedKalmanFilter ekf;
unsigned long prevTime = 0;

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

    delay(2000);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    
    prevTime = micros();
    Serial.println("초기화 완료!");
}

void loop() {
    if (mpu.update()) {
        unsigned long currentTime = micros();
        float dt = (currentTime - prevTime) / 1000000.0f;
        ekf.setDt(dt);
        
        float gx = mpu.getGyroX() * D2R;
        float gy = mpu.getGyroY() * D2R;
        float gz = mpu.getGyroZ() * D2R;
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        // 자기장 데이터 의 오류로 6축만 사용
        //float mx = mpu.getMagX();
        //float my = mpu.getMagY();
        //float mz = mpu.getMagZ();
        
        ekf.predict(gx, gy, gz);
        ekf.update(ax, ay, az);
        
        static uint32_t print_ms = millis();
        if (millis() - print_ms >= 25) {
            float roll, pitch, yaw;
            ekf.getEulerAngles(roll, pitch, yaw);
            
            Serial.print(roll, 2);
            Serial.print(",");
            Serial.print(pitch, 2);
            Serial.print(",");
            Serial.println(yaw, 2);
                
            print_ms = millis();
        }
        
        prevTime = currentTime;
    }
}