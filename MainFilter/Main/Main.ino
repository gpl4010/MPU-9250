#include <Wire.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include "esp_timer.h"
#include <freertos/task.h>

// MPU9250 레지스터 주소 설정
#define MPU9250_ADDRESS     0x68
#define MAG_ADDRESS         0x0C

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

// I2C LCD 객체 생성
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastLcdUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 1000;

// Bluetooth 시리얼 객체 생성
BluetoothSerial SerialBT;

// LED PWM 핀 정의
const int PWM_PIN = 2;
int pwmValue = 0;

// 스위치 핀 정의
const int CALIBRATION_SWITCH = 12;
const int DISPLAY_SWITCH = 13;
volatile unsigned long lastDisplaySwitchTime = 0;
const unsigned long DEBOUNCE_TIME = 200;

// 전역 변수
float gyroBias[3] = {0, 0, 0};
float accBias[3] = {0, 0, 0};
float magBias[3] = {0, 0, 0};
int displayMode = 0;
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
volatile bool calibrateFlag = false;

// 태스크 핸들
TaskHandle_t IMUTaskHandle;
esp_timer_handle_t timer;

class MyFilter {
private:
    float beta;
    float q0, q1, q2, q3;
    float sampleFreq;
    
public:
    MyFilter() {
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
    float R11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float R21 = 2.0f * (q1 * q2 + q0 * q3);
    float R31 = 2.0f * (q1 * q3 - q0 * q2);
    float R32 = 2.0f * (q2 * q3 + q0 * q1);
    float R33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    *roll = atan2(R32, R33) * 180.0f / PI;
    *pitch = atan2(-R31, sqrt(R32 * R32 + R33 * R33)) * 180.0f / PI;
    *yaw = atan2(R21, R11) * 180.0f / PI;
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
    float accSumX = 0, accSumY = 0, accSumZ = 0;
    float magSumX = 0, magSumY = 0, magSumZ = 0;
    const int samples = 1000;
    
    Serial.println("센서 초기화를 진행 중 입니다...");
    Serial.println("센서를 움직이지 마세요");
    
    // 자이로, 가속도 데이터 샘플링
    for(int i = 0; i < samples; i++) {
        uint8_t Buf[14];
        I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
        
        // 가속도 데이터
        int16_t ax = (Buf[0] << 8) | Buf[1];
        int16_t ay = (Buf[2] << 8) | Buf[3];
        int16_t az = (Buf[4] << 8) | Buf[5];
        
        // 자이로 데이터
        int16_t gx = (Buf[8] << 8) | Buf[9];
        int16_t gy = (Buf[10] << 8) | Buf[11];
        int16_t gz = (Buf[12] << 8) | Buf[13];
        
        accSumX += ax;
        accSumY += ay;
        accSumZ += az;
        
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        
        //진행도 출력
        if(i % 100 == 0) {
            Serial.print("Progress: ");
            Serial.print((i * 100) / samples);
            Serial.println("%");
        }
        
        delay(1);
    }
    
    // 지자기 센서 데이터 수집
    for(int i = 0; i < samples/10; i++) {  // 지자기 센서는 더 낮은 샘플링 레이트
        uint8_t ST1;
        I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
        
        if (ST1 & 0x01) {
            uint8_t Mag[7];
            I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
            
            int16_t mx = (Mag[1] << 8) | Mag[0];
            int16_t my = (Mag[3] << 8) | Mag[2];
            int16_t mz = (Mag[5] << 8) | Mag[4];
            
            magSumX += mx;
            magSumY += my;
            magSumZ += mz;
        }
        delay(10);
    }
    
    // 평균 계산하여 바이어스 값 설정
    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;
    
    accBias[0] = accSumX / samples;
    accBias[1] = accSumY / samples;
    accBias[2] = (accSumZ / samples) - (32768 / ACC_SCALE); // 중력 가속도 보정
    
    magBias[0] = magSumX / (samples/10);
    magBias[1] = magSumY / (samples/10);
    magBias[2] = magSumZ / (samples/10);
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

struct SharedData {
    float roll;
    float pitch;
    float yaw;
    portMUX_TYPE mux;
} sharedData = {0, 0, 0, portMUX_INITIALIZER_UNLOCKED};

// 타이머 인터럽트 핸들러
void IRAM_ATTR onTimer(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(IMUTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// IMU Task
void IMUTask(void* parameter) {
    static uint32_t prev_ms = 0;
    static float mx = 0, my = 0, mz = 0;
    
    while(true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        uint32_t now = millis();
        float dt = (now - prev_ms) / 1000.0f;

        // 센서 데이터 읽기
        uint8_t Buf[14];
        I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
        
        // 가속도, 자이로 데이터 처리
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
        
        // 지자기 데이터 업데이트 (100ms마다)
        static uint32_t mag_update = 0;
        if (now - mag_update >= 100) {
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
        
        // 필터 업데이트
        filter.update(gx_rads, gy_rads, gz_rads, ax_ms2, ay_ms2, az_ms2, mx, my, mz, dt);
        
        // 각도 계산
        float roll, pitch, yaw;
        filter.getAngles(&roll, &pitch, &yaw);
        
        // 이동 평균 필터
        #define FILTER_SIZE 5
        static float roll_array[FILTER_SIZE] = {0};
        static float pitch_array[FILTER_SIZE] = {0};
        static float yaw_array[FILTER_SIZE] = {0};
        static int filter_index = 0;
        
        roll_array[filter_index] = roll - offsetRoll;
        pitch_array[filter_index] = pitch - offsetPitch;
        yaw_array[filter_index] = yaw - offsetYaw;
        filter_index = (filter_index + 1) % FILTER_SIZE;
        
        float roll_filtered = 0, pitch_filtered = 0, yaw_filtered = 0;
        for(int i = 0; i < FILTER_SIZE; i++) {
            roll_filtered += roll_array[i];
            pitch_filtered += pitch_array[i];
            yaw_filtered += yaw_array[i];
        }
        
        portENTER_CRITICAL(&sharedData.mux);
        sharedData.roll = roll_filtered / FILTER_SIZE;
        sharedData.pitch = pitch_filtered / FILTER_SIZE;
        sharedData.yaw = yaw_filtered / FILTER_SIZE;
        portEXIT_CRITICAL(&sharedData.mux);
        
        // PWM 값 업데이트
        switch(displayMode) {
            case 0: pwmValue = map(abs((int)sharedData.roll), 0, 180, 0, 255); break;
            case 1: pwmValue = map(abs((int)sharedData.pitch), 0, 180, 0, 255); break;
            case 2: pwmValue = map(abs((int)sharedData.yaw), 0, 180, 0, 255); break;
        }
        analogWrite(PWM_PIN, pwmValue);
        
        // 시리얼 및 블루투스 출력
        String output = String("Roll: ") + String(sharedData.roll, 2) +
                       " Pitch: " + String(sharedData.pitch, 2) +
                       " Yaw: " + String(sharedData.yaw, 2);
        
        Serial.println(output);
        if (SerialBT.hasClient()) {
            SerialBT.println(output);
        }
        
        // LCD 업데이트
        if (now - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
            lastLcdUpdate = now;
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Angle:");
            lcd.setCursor(0, 1);
            
            switch(displayMode) {
                case 0:
                    lcd.print("Roll: ");
                    lcd.print(sharedData.roll, 2);
                    break;
                case 1:
                    lcd.print("Pitch: ");
                    lcd.print(sharedData.pitch, 2);
                    break;
                case 2:
                    lcd.print("Yaw: ");
                    lcd.print(sharedData.yaw, 2);
                    break;
            }
        }
        

        // 캘리브레이션 처리
        if (calibrateFlag) {
            offsetRoll = roll;
            offsetPitch = pitch;
            offsetYaw = yaw;
            
            lcd.clear();
            lcd.print("Calibrated");
            Serial.println("Calibrated");
            vTaskDelay(pdMS_TO_TICKS(1000)); //이거 대신 자이로 캘리브레이션을 넣으면 더 좋지 않을까 싶음
            calibrateFlag = false;
        }
        
        prev_ms = now;
    }
}

void IRAM_ATTR switchISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    calibrateFlag = true;
    vTaskNotifyGiveFromISR(IMUTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR displaySwitchISR() {
    unsigned long currentTime = millis();
    if (currentTime - lastDisplaySwitchTime > DEBOUNCE_TIME) {
        displayMode = (displayMode + 1) % 3;  // 0, 1, 2 순환
        lastDisplaySwitchTime = currentTime;
    }
}

//serial 내 조작
void printHelp(Stream& stream) {
    stream.println("Available commands:");
    stream.println("r: Set display mode to Roll");
    stream.println("p: Set display mode to Pitch");
    stream.println("y: Set display mode to Yaw");
    stream.println("c: Trigger calibration");
    stream.println("h: Show this help message");
}

void handleCommand(char cmd, bool isSerial) {
    Stream* stream;
    if (isSerial) {
        stream = &Serial;
        stream = &SerialBT;
    } else {
    }

    switch (cmd) {
        case 'r':
        case 'R':
            displayMode = 0; // Roll
            stream->println("Display mode set to Roll");
            break;
        case 'p':
        case 'P':
            displayMode = 1; // Pitch
            stream->println("Display mode set to Pitch");
            break;
        case 'y':
        case 'Y':
            displayMode = 2; // Yaw
            stream->println("Display mode set to Yaw");
            break;
        case 'c':
        case 'C':
            calibrateFlag = true;
            stream->println("Calibration triggered");
            break;
        case 'h':
        case 'H':
            printHelp(*stream);
            break;
        default:
            stream->println("Unknown command. Type 'h' for help.");
    }
}

void setup() {
    // 통신 초기화
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    
    // 블루투스 초기화
    if (!SerialBT.begin("ESP32_IMU")) {
        Serial.println("Bluetooth initialization failed");
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    // LCD 초기화
    lcd.init();
    lcd.backlight();
    lcd.print("Initializing...");
    
    // MPU9250 초기화
    initMPU9250();
    delay(1000);
    
    // 자이로 캘리브레이션
    calibrateGyro();
    
    // 핀 연결 설정
    pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
    pinMode(DISPLAY_SWITCH, INPUT_PULLUP);
    pinMode(PWM_PIN, OUTPUT);

    //스위치 인터럽트 설정
    attachInterrupt(digitalPinToInterrupt(CALIBRATION_SWITCH), switchISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(DISPLAY_SWITCH), displaySwitchISR, FALLING);
    
    // 타이머 설정
    const esp_timer_create_args_t timer_args = {
        .callback = &onTimer,
        .name = "imu_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 40000)); // 40ms
    
    // 태스크 생성
    xTaskCreatePinnedToCore(
        IMUTask,
        "IMUTask",
        4096,
        NULL,
        1,
        &IMUTaskHandle,
        0
    );
    
    Serial.println("시스템 초기화. 'H'/'h'를 통해 키 확인");
    lcd.clear();
    lcd.print("Ready");
}

void loop() {
    if (Serial.available()) {
        handleCommand(Serial.read(), true);
    }
    if (SerialBT.available()) {
        handleCommand(SerialBT.read(), false);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}