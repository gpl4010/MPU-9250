#include <Wire.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
    else {
      Serial.println("Error: Failed to update IMU data");
      SerialBT.println("Error: Failed to update IMU data");
      lcd.clear();
      lcd.print("IMU Error");
      vTaskDelay(pdMS_TO_TICKS(1000)); // 1초 대기
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
  if (!SerialBT.begin("ESP32_IMU")) {
    Serial.println("Error: Bluetooth initialization failed");
    while (1) {
      delay(1000);
    }
  }
  
  Wire.begin();
  
  // MPU9250 초기화
  if (!IMU.setup(0x68)) {
    Serial.println("Error: MPU9250 initialization failed");
    lcd.init();
    lcd.backlight();
    lcd.print("MPU9250 Error");
    while (1) {
      delay(1000);
    }
  }
  
  // Madgwick 필터 초기화
  filter.begin(100); // 100Hz 샘플링 레이트
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  
  // 스위치 핀 설정
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switchISR, FALLING);
  
  // PWM 핀 설정코드 추가 필요
  
  // IMU 태스크 생성
  BaseType_t xReturned = xTaskCreatePinnedToCore(
    IMUTask,        // 태스크 함수
    "IMUTask",      // 태스크 이름
    10000,          // 스택 크기
    NULL,           // 태스크 파라미터
    1,              // 태스크 우선순위
    &IMUTaskHandle, // 태스크 핸들
    0               // 실행할 코어 (0 or 1)
  );

  if (xReturned != pdPASS) {
    Serial.println("Error: Failed to create IMU task");
    while (1) {}
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}