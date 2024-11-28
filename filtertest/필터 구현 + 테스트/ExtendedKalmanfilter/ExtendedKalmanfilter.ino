#include <MPU9250.h>
#include <Wire.h>
#include "eigen.h"

using namespace Eigen;

const float D2R = M_PI/180.0f;
const float R2D = 180.0f/M_PI;

MPU9250 mpu;

class ExtendedKalmanFilter {
private:
    Matrix<float, 6, 1> x_state;
    Matrix<float, 6, 6> P_cov;    
    Matrix<float, 6, 6> Q_noise;  
    Matrix<float, 3, 3> R_noise;  
    float dt;

    // 3x3 역행렬 계산 함수
    Matrix<float, 3, 3> inverse3x3(const Matrix<float, 3, 3>& matrix) {
        Matrix<float, 3, 3> result;
        float det = matrix(0,0) * (matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1)) -
                   matrix(0,1) * (matrix(1,0) * matrix(2,2) - matrix(1,2) * matrix(2,0)) +
                   matrix(0,2) * (matrix(1,0) * matrix(2,1) - matrix(1,1) * matrix(2,0));
        
        float invDet = 1.0f / det;
        
        result(0,0) = (matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1)) * invDet;
        result(0,1) = (matrix(0,2) * matrix(2,1) - matrix(0,1) * matrix(2,2)) * invDet;
        result(0,2) = (matrix(0,1) * matrix(1,2) - matrix(0,2) * matrix(1,1)) * invDet;
        result(1,0) = (matrix(1,2) * matrix(2,0) - matrix(1,0) * matrix(2,2)) * invDet;
        result(1,1) = (matrix(0,0) * matrix(2,2) - matrix(0,2) * matrix(2,0)) * invDet;
        result(1,2) = (matrix(0,2) * matrix(1,0) - matrix(0,0) * matrix(1,2)) * invDet;
        result(2,0) = (matrix(1,0) * matrix(2,1) - matrix(1,1) * matrix(2,0)) * invDet;
        result(2,1) = (matrix(0,1) * matrix(2,0) - matrix(0,0) * matrix(2,1)) * invDet;
        result(2,2) = (matrix(0,0) * matrix(1,1) - matrix(0,1) * matrix(1,0)) * invDet;
        
        return result;
    }

public:
    ExtendedKalmanFilter() : dt(0.01f) { // 100Hz default
        // Initialize state vector
        x_state.setZero();

        // Initialize covariance matrices
        P_cov.setIdentity();
        P_cov *= 0.1f;

        // Process noise
        Q_noise.setIdentity();
        Q_noise.block<3,3>(0,0) *= 0.001f;  // Angle states
        Q_noise.block<3,3>(3,3) *= 0.003f;  // Bias states

        // Measurement noise
        R_noise.setIdentity();
        R_noise *= 0.3f;
    }

    void predict(float gx, float gy, float gz) {
        // Correct gyro with estimated bias
        float wx = gx - x_state(3);
        float wy = gy - x_state(4);
        float wz = gz - x_state(5);

        // State prediction
        Matrix<float, 6, 1> x_pred;
        x_pred(0) = x_state(0) + dt * (wx + (wy * sin(x_state(0)) + wz * cos(x_state(0))) * tan(x_state(1)));
        x_pred(1) = x_state(1) + dt * (wy * cos(x_state(0)) - wz * sin(x_state(0)));
        x_pred(2) = x_state(2) + dt * ((wy * sin(x_state(0)) + wz * cos(x_state(0))) / cos(x_state(1)));
        x_pred(3) = x_state(3);
        x_pred(4) = x_state(4);
        x_pred(5) = x_state(5);

        // Jacobian of state transition
        Matrix<float, 6, 6> F_jacob;
        F_jacob.setIdentity();
        
        float sp = sin(x_state(0));
        float cp = cos(x_state(0));
        float tt = tan(x_state(1));
        float ct = cos(x_state(1));

        F_jacob(0,0) = 1.0f + dt * (wy * cp - wz * sp) * tt;
        F_jacob(0,1) = dt * (wy * sp + wz * cp) / (ct * ct);
        F_jacob(1,0) = dt * (-wy * sp - wz * cp);
        F_jacob(2,0) = dt * (wy * cp - wz * sp) / ct;
        F_jacob(2,1) = dt * (wy * sp + wz * cp) * sp / (ct * ct);

        // Covariance prediction
        P_cov = F_jacob * P_cov * F_jacob.transpose() + Q_noise;
        
        // Update state
        x_state = x_pred;
    }

    void update(float ax, float ay, float az, float mx, float my, float mz) {
        // Normalize accelerometer data
        float acc_norm = sqrt(ax*ax + ay*ay + az*az);
        if(acc_norm > 0.01f) {
            ax /= acc_norm;
            ay /= acc_norm;
            az /= acc_norm;
        }

        // Calculate roll and pitch from accelerometer
        float roll_m = atan2(ay, az);
        float pitch_m = atan2(-ax, sqrt(ay*ay + az*az));

        // Tilt compensate magnetometer
        float cos_roll = cos(roll_m);
        float sin_roll = sin(roll_m);
        float cos_pitch = cos(pitch_m);
        float sin_pitch = sin(pitch_m);

        float Xh = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
        float Yh = my * cos_roll - mz * sin_roll;

        float yaw_m = atan2(-Yh, Xh);

        // Measurement vector
        Matrix<float, 3, 1> z_meas;
        z_meas << roll_m, pitch_m, yaw_m;

        // Measurement model Jacobian
        Matrix<float, 3, 6> H_jacob;
        H_jacob.setZero();
        H_jacob.block<3,3>(0,0).setIdentity();

        // Innovation
        Matrix<float, 3, 1> y = z_meas - H_jacob * x_state;
        
        // Normalize yaw innovation
        while(y(2) > M_PI) y(2) -= 2*M_PI;
        while(y(2) < -M_PI) y(2) += 2*M_PI;

        // Innovation covariance
        Matrix<float, 3, 3> S = H_jacob * P_cov * H_jacob.transpose() + R_noise;

        // Kalman gain using manual inverse
        Matrix<float, 6, 3> K = P_cov * H_jacob.transpose() * inverse3x3(S);

        // State update
        x_state = x_state + K * y;

        // Covariance update
        Matrix<float, 6, 6> I = Matrix<float, 6, 6>::Identity();
        P_cov = (I - K * H_jacob) * P_cov;
    }

    void getEulerAngles(float& roll, float& pitch, float& yaw) {
        roll = x_state(0) * R2D;
        pitch = x_state(1) * R2D;
        yaw = x_state(2) * R2D;
        
        if(yaw < 0) yaw += 360.0f;
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
    
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU9250 connection failed!");
            delay(5000);
        }
    }

    Serial.println("Calibrating sensors...");
    delay(2000);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    
    prevTime = micros();
    Serial.println("Setup complete!");
}

void loop() {
    if (mpu.update()) {
        unsigned long currentTime = micros();
        float dt = (currentTime - prevTime) / 1000000.0f;
        ekf.setDt(dt);
        
        // Get sensor data
        float gx = mpu.getGyroX() * D2R;
        float gy = mpu.getGyroY() * D2R;
        float gz = mpu.getGyroZ() * D2R;
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        float mx = mpu.getMagX();
        float my = mpu.getMagY();
        float mz = mpu.getMagZ();
        
        // EKF update
        ekf.predict(gx, gy, gz);
        ekf.update(ax, ay, az, mx, my, mz);
        
        // Print results every 25ms
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            float roll, pitch, yaw;
            ekf.getEulerAngles(roll, pitch, yaw);
            
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