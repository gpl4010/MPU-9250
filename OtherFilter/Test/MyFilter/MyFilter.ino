#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;

const float GYRO_SCALE = (2000.0f / 32768.0f) * (PI / 180.0f);
const float ACC_SCALE = (4.0f / 32768.0f) * 9.81f;
const float MAG_SCALE = (4912.0f / 32760.0f);

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

// 자이로 바이어스 저장용
float gyroBias[3] = {0, 0, 0};

// 자이로스코프 드리프트 방지
void calibrateGyro() {
    float sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 1000;
    
    Serial.println("Calibrating gyroscope... Keep the sensor still!");
    
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
    Serial.print("Bias X: "); Serial.println(gyroBias[0]);
    Serial.print("Bias Y: "); Serial.println(gyroBias[1]);
    Serial.print("Bias Z: "); Serial.println(gyroBias[2]);
}

MyFilter filter;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU9250 connection failed!");
            delay(5000);
        }
    }

    Serial.println("Calibrating sensors...");
    delay(2000);
    
    // 자이로 캘리브레이션 수행
    calibrateGyro();
    
    Serial.println("Setup complete!");
}

void loop() {
    static uint32_t prev_ms = millis();
    
    if (mpu.update()) {
        uint32_t now = millis();
        float dt = (now - prev_ms) / 1000.0f;
        
        if (now - prev_ms >= 20) {  // 50Hz
            // 센서 데이터 읽기 및 변환
            float gx = (mpu.getGyroX() - gyroBias[0]) * DEG_TO_RAD;  // 바이어스 보정 추가
            float gy = (mpu.getGyroY() - gyroBias[1]) * DEG_TO_RAD;
            float gz = (mpu.getGyroZ() - gyroBias[2]) * DEG_TO_RAD;
            
            float ax = mpu.getAccX();
            float ay = mpu.getAccY();
            float az = mpu.getAccZ();
            
            float mx = mpu.getMagX() * MAG_SCALE;
            float my = mpu.getMagY() * MAG_SCALE;
            float mz = mpu.getMagZ() * MAG_SCALE;
            
            filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
            
            float roll, pitch, yaw;
            filter.getAngles(&roll, &pitch, &yaw);
            
            //가속도계 노이즈 제거
            #define FILTER_SIZE 5
            static float roll_array[FILTER_SIZE] = {0};
            static float pitch_array[FILTER_SIZE] = {0};
            static float yaw_array[FILTER_SIZE] = {0};
            static int filter_index = 0;
            
            roll_array[filter_index] = roll;
            pitch_array[filter_index] = pitch;
            yaw_array[filter_index] = yaw;
            filter_index = (filter_index + 1) % FILTER_SIZE;
            
            float roll_filtered = 0, pitch_filtered = 0, yaw_filtered = 0;
            for(int i = 0; i < FILTER_SIZE; i++) {
                roll_filtered += roll_array[i];
                pitch_filtered += pitch_array[i];
                yaw_filtered += yaw_array[i];
            }
            roll_filtered /= FILTER_SIZE;
            pitch_filtered /= FILTER_SIZE;
            yaw_filtered /= FILTER_SIZE;
            

            static uint32_t print_ms = millis();
            if (now - print_ms >= 50) {  // 20Hz로 출력
                Serial.print(roll_filtered, 2);
                Serial.print(",");
                Serial.print(pitch_filtered, 2);
                Serial.print(",");
                Serial.println(yaw_filtered, 2);
                
                print_ms = now;
            }
            
            prev_ms = now;
        }
    }
}