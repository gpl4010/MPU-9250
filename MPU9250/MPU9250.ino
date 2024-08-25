#include <Wire.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_timer.h"

// MPU9250 센서 객체 생성
MPU9250 IMU;

// Madgwick 필터 객체 생성
Madgwick filter;

// I2C LCD 객체 생성 (주소 0x27, 16열 2행)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Bluetooth 시리얼 객체 생성
BluetoothSerial SerialBT;

// 스위치 핀 정의
const int SWITCH_PIN = 14;

// PWM 핀 정의
const int PWM_PIN = 27;

// PWM 설정
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

// 전역 변수
float roll, pitch, yaw;
int displayMode = 0; // 0: roll, 1: pitch, 2: yaw
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
volatile bool calibrateFlag = false;

// 태스크 핸들
TaskHandle_t IMUTaskHandle;

// 타이머 핸들
esp_timer_handle_t timer;

void IRAM_ATTR onTimer(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(IMUTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void IMUTask(void * parameter) {
    while(true) {
        // 타이머 인터럽트를 기다립니다
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (IMU.update()) {
            // IMU 데이터 읽기 및 필터링
            float gx = IMU.getGyroX();
            float gy = IMU.getGyroY();
            float gz = IMU.getGyroZ();
            float ax = IMU.getAccX();
            float ay = IMU.getAccY();
            float az = IMU.getAccZ();
            
            filter.updateIMU(gx, gy, gz, ax, ay, az);
            
            roll = filter.getRoll() - offsetRoll;
            pitch = filter.getPitch() - offsetPitch;
            yaw = filter.getYaw() - offsetYaw;
            
            // PWM 값 계산 (예: roll 각도에 비례)
            int pwmValue = map(abs((int)roll), 0, 180, 0, 255);
            ledcWrite(PWM_CHANNEL, pwmValue);
            
            // UART 출력
            Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, PWM: %d\n", roll, pitch, yaw, pwmValue);
            
            // Bluetooth 출력
            if (SerialBT.hasClient()) {
                SerialBT.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, PWM: %d\n", roll, pitch, yaw, pwmValue);
            }
            
            // LCD 표시
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Angle:");
            lcd.setCursor(0, 1);
            switch (displayMode) {
                case 0:
                    lcd.print("Roll: ");
                    lcd.print(roll, 2);
                    break;
                case 1:
                    lcd.print("Pitch: ");
                    lcd.print(pitch, 2);
                    break;
                case 2:
                    lcd.print("Yaw: ");
                    lcd.print(yaw, 2);
                    break;
            }
        }
        else {
            Serial.println("Error: Failed to update IMU data");
            if (SerialBT.hasClient()) {
                SerialBT.println("Error: Failed to update IMU data");
            }
            lcd.clear();
            lcd.print("IMU Error");
        }

        if (calibrateFlag) {
            offsetRoll = filter.getRoll();
            offsetPitch = filter.getPitch();
            offsetYaw = filter.getYaw();

            lcd.clear();
            lcd.print("Calibrated");
            vTaskDelay(pdMS_TO_TICKS(1000));
            calibrateFlag = false;
        }
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
    } else {
        stream = &SerialBT;
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
    Serial.begin(115200);
    
    if (!SerialBT.begin("ESP32_IMU")) {
        Serial.println("Error: Bluetooth initialization failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    Wire.begin();
    
    // LCD 초기화
    lcd.init();
    lcd.backlight();
    
    // MPU9250 초기화
    if (!IMU.setup(0x68)) {
        Serial.println("Error: MPU9250 initialization failed");
        lcd.print("MPU9250 Error");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    // Madgwick 필터 초기화
    filter.begin(100); // 100Hz 샘플링 레이트
    
    // 스위치 핀 설정
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switchISR, FALLING);

    // PWM 설정
    //ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    //ledcAttachPin(PWM_PIN, PWM_CHANNEL);
    
    // 40ms 타이머 설정
    const esp_timer_create_args_t timer_args = {
        .callback = &onTimer,
        .name = "imu_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 40000));
    
    //안내 메시지 출력
    Serial.println("IMU System initialized. Type 'h' for available commands.");
    SerialBT.println("IMU System initialized. Type 'h' for available commands.");

    // IMU 태스크 생성
    xTaskCreatePinnedToCore(
        IMUTask,
        "IMUTask",      
        4096,           
        NULL,           
        1,              
        &IMUTaskHandle, 
        0               
    );
}

void loop() {
  if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd, true);
    }
    if (SerialBT.available()) {
        char cmd = SerialBT.read();
        handleCommand(cmd, false);
    }
  vTaskDelay(pdMS_TO_TICKS(1000));
}