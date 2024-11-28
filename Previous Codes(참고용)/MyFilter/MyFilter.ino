#include <Wire.h>
#include <math.h>

// MPU9250 레지스터 주소
#define MPU9250_ADDRESS     0x68
#define MAG_ADDRESS         0x0C

// MPU9250 레지스터 맵
#define PWR_MGMT_1         0x6B
#define CONFIG             0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define ACCEL_CONFIG2      0x1D
#define SMPLRT_DIV         0x19
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define MAG_CNTL1          0x0A
#define MAG_CNTL2          0x0B

// 스케일 팩터
#define GYRO_SCALE      (2000.0f / 32768.0f) * (PI / 180.0f)  // LSB -> rad/s
#define ACC_SCALE       (4.0f / 32768.0f) * 9.81f             // LSB -> m/s^2
#define MAG_SCALE       (4912.0f / 32760.0f)                  // LSB -> uT

// 전역 변수
float gyroBias[3] = {0, 0, 0};
float accBias[3] = {0, 0, 0};
float magBias[3] = {0, 0, 0};

class MyFilter {
private:
    float beta;
    float q0, q1, q2, q3;
    float sampleFreq;
    
public:
    MyFilter() {
        beta = 0.1f;
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

            // 정규화
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

        // 정규화
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

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

void calibrateGyro() {
    float sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 1000;
    
    Serial.println("Calibrating gyroscope... Keep the sensor still!");
    
    for(int i = 0; i < samples; i++) {
        uint8_t Buf[14];
        I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
        
        int16_t gx = (Buf[8] << 8) | Buf[9];
        int16_t gy = (Buf[10] << 8) | Buf[11];
        int16_t gz = (Buf[12] << 8) | Buf[13];
        
        sumX += gx;
        sumY += gy;
        sumZ += gz;
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

void initMPU9250() {
    // MPU9250 리셋
    I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    delay(100);
    
    // 클럭 소스 선택
    I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    delay(200);
    
    // 설정
    I2CwriteByte(MPU9250_ADDRESS, CONFIG, 0x03);         // DLPF_CFG = 3
    I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x18);    // +-2000dps
    I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x08);   // +-4g
    I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);  // DLPF_CFG = 3
    I2CwriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);    // 200Hz
    
    // 자기센서 설정
    I2CwriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);   // Bypass Enable
    delay(10);
    
    I2CwriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00);         // Power down
    delay(10);
    I2CwriteByte(MAG_ADDRESS, MAG_CNTL1, 0x16);         // 16bit, 100Hz
    delay(10);
}

MyFilter filter;

void setup() {
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    Serial.begin(115200);
    
    Serial.println("Initializing MPU9250...");
    initMPU9250();
    delay(1000);
    
    Serial.println("Starting gyro calibration...");
    calibrateGyro();
    
    Serial.println("Initialization complete!");
}

void loop() {
    static uint32_t prev_ms = millis();
    uint32_t now = millis();
    float dt = (now - prev_ms) / 1000.0f;
    
    if (now - prev_ms >= 20) {  // 50Hz
        uint8_t Buf[14];
        I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
        
        // 센서 데이터 읽기
        int16_t ax = (Buf[0] << 8) | Buf[1];
        int16_t ay = (Buf[2] << 8) | Buf[3];
        int16_t az = (Buf[4] << 8) | Buf[5];
        int16_t gx = (Buf[8] << 8) | Buf[9];
        int16_t gy = (Buf[10] << 8) | Buf[11];
        int16_t gz = (Buf[12] << 8) | Buf[13];
        
        // 스케일 변환
        float ax_ms2 = -(float)ax * ACC_SCALE;
        float ay_ms2 = -(float)ay * ACC_SCALE;
        float az_ms2 = (float)az * ACC_SCALE;
        
        float gx_rads = -(float)(gx - gyroBias[0]) * GYRO_SCALE;
        float gy_rads = -(float)(gy - gyroBias[1]) * GYRO_SCALE;
        float gz_rads = (float)(gz - gyroBias[2]) * GYRO_SCALE;
        
        // 지자기 데이터 읽기
        static uint32_t mag_update = millis();
        static float mx = 0, my = 0, mz = 0;
        
        if (now - mag_update >= 100) {  // 10Hz
            uint8_t ST1;
            I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
            
            if (ST1 & 0x01) {
                uint8_t Mag[7];
                I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
                
                int16_t mx_raw = (Mag[1] << 8) | Mag[0];
                int16_t my_raw = (Mag[3] << 8) | Mag[2];
                int16_t mz_raw = (Mag[5] << 8) | Mag[4];
                
                mx = (float)mx_raw * MAG_SCALE;
                my = (float)my_raw * MAG_SCALE;
                mz = (float)mz_raw * MAG_SCALE;
                mag_update = now;
            }
        }
        
        // My 필터 업데이트
        filter.update(gx_rads, gy_rads, gz_rads, ax_ms2, ay_ms2, az_ms2, mx, my, mz, dt);
        
        // 각도 계산
        float roll, pitch, yaw;
        filter.getAngles(&roll, &pitch, &yaw);
        
        // 이동 평균 필터 적용
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
        
        // 결과 출력
        static uint32_t print_ms = millis();
        if (now - print_ms >= 50) {  // 20Hz로 출력
            Serial.print("Roll: ");
            Serial.print(roll_filtered, 2);
            Serial.print(" Pitch: ");
            Serial.print(pitch_filtered, 2);
            Serial.print(" Yaw: ");
            Serial.println(yaw_filtered, 2);
            
            print_ms = now;
        }
        
        prev_ms = now;
    }
}