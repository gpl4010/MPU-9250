#include <Wire.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// MPU9250 센서 객체 생성
MPU9250 mpu;

// Madgwick 필터 객체 생성
Madgwick filter;

// I2C LCD 객체 생성 (주소 0x27, 16열 2행)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Bluetooth 시리얼 객체 생성
BluetoothSerial SerialBT;

// 스위치 핀 정의
const int SWITCH_PIN = 4;

// PWM 핀 정의
const int PWM_PIN = 5;

// 전역 변수
float roll, pitch, yaw;
int displayMode = 0; // 0: roll, 1: pitch, 2: yaw
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
volatile bool calibrateFlag = false;

// 태스크 핸들
TaskHandle_t IMUTaskHandle;

void IMUTask(void * parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(40);

  xLastWakeTime = xTaskGetTickCount();

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (mpu.update()) {
      // IMU 데이터 읽기 및 필터링
      float gx = mpu.getGyroX();
      float gy = mpu.getGyroY();
      float gz = mpu.getGyroZ();
      float ax = mpu.getAccX();
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();
      
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      
      roll = filter.getRoll() - offsetRoll;
      pitch = filter.getPitch() - offsetPitch;
      yaw = filter.getYaw() - offsetYaw;
      
      // PWM 값 계산 (예: roll 각도에 비례)
      int pwmValue = map(abs((int)roll), 0, 180, 0, 255);
      ledcWrite(0, pwmValue);
      
      // UART 출력
      Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, PWM: %d\n", roll, pitch, yaw, pwmValue);
      
      // Bluetooth 출력
      SerialBT.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, PWM: %d\n", roll, pitch, yaw, pwmValue);
      
      // LCD 표시
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Angle:");
      lcd.setCursor(0, 1);
      switch (displayMode) {
        case 0:
          lcd.printf("Roll: %.2f", roll);
          break;
        case 1:
          lcd.printf("Pitch: %.2f", pitch);
          break;
        case 2:
          lcd.printf("Yaw: %.2f", yaw);
          break;
      }
    }

    if (calibrateFlag) {
      offsetRoll = filter.getRoll();
      offsetPitch = filter.getPitch();
      offsetYaw = filter.getYaw();
      
      lcd.clear();
      lcd.print("Calibrated");
      delay(1000);
      calibrateFlag = false;
    }
  }
}

void IRAM_ATTR switchISR() {
  calibrateFlag = true;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_IMU");
  Wire.begin();
  
  // MPU9250 초기화
  mpu.setup(0x68);
  
  // Madgwick 필터 초기화
  filter.begin(100); // 100Hz 샘플링 레이트
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  
  // 스위치 핀 설정
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switchISR, FALLING);
  
  // PWM 핀 설정
  ledcSetup(0, 5000, 8); // 채널 0, 5kHz PWM, 8-bit 해상도
  ledcAttachPin(PWM_PIN, 0);
  
  // IMU 태스크 생성
  xTaskCreatePinnedToCore(
    IMUTask,        // 태스크 함수
    "IMUTask",      // 태스크 이름
    10000,          // 스택 크기
    NULL,           // 태스크 파라미터
    1,              // 태스크 우선순위
    &IMUTaskHandle, // 태스크 핸들
    0               // 실행할 코어 (0 or 1)
  );
}

void loop() {
  // 메인 루프는 비어 있음
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}