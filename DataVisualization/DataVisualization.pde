// "Roll: 0.00, Pitch: 0.00, Yaw: 0.00" 형식 파싱
// MPU9250.ino파일용

import processing.serial.*;

Serial myPort;

// 데이터 저장
ArrayList<Float> rollData = new ArrayList<Float>();
ArrayList<Float> pitchData = new ArrayList<Float>();
ArrayList<Float> yawData = new ArrayList<Float>();

// 현재 각도 값
float roll = 0;
float pitch = 0;
float yaw = 0;

// 그래프 설정
int maxDataPoints = 200;
float yMin = -180;
float yMax = 180;
float leftMargin = 60;
float rightMargin = 30;
float topMargin = 30;
float bottomMargin = 30;
float subplotSpacing = 20;

// 색상 설정
color textColor = color(0);
color gridColor = color(200);
color backgroundColor = color(240);

// 3D 설정
float cameraRotX = -0.4;
float cameraRotY = 0.6;

// 시각화 모드
boolean is3DMode = false;

void setup() {
    size(1800, 1000, P3D);  // P3D 렌더러 사용
    
    String[] ports = Serial.list();
    println("사용 가능한 시리얼 포트 목록:");
    for (int i = 0; i < ports.length; i++) {
        println(i + ": " + ports[i]);
    }
    
    try {
        myPort = new Serial(this, Serial.list()[2], 115200);
        myPort.bufferUntil('\n');
        println("포트 연결 성공: " + Serial.list()[2]);
    } catch (Exception e) {
        println("포트 연결 실패! 포트 번호를 확인하세요.");
        println("에러: " + e.getMessage());
        exit();
    }
    
    textSize(18);
}

void draw() {
    background(backgroundColor);
    
    if (is3DMode) {
        draw3D();
    } else {
        draw2D();
    }
    
    // 모드 전환 안내 텍스트
    fill(textColor);
    textAlign(LEFT, BOTTOM);
    text("Space키: 2D/3D 모드 전환", 10, height - 10);
}

void draw2D() {
    float subplotHeight = (height - topMargin - bottomMargin - 2 * subplotSpacing) / 3;
    float graphWidth = width - leftMargin - rightMargin;
    
    drawSubplot(rollData, "Roll (degrees)", color(255, 0, 0), 
               leftMargin, topMargin, 
               graphWidth, subplotHeight);
               
    drawSubplot(pitchData, "Pitch (degrees)", color(0, 255, 0),
               leftMargin, topMargin + subplotHeight + subplotSpacing,
               graphWidth, subplotHeight);
               
    drawSubplot(yawData, "Yaw (degrees)", color(0, 0, 255),
               leftMargin, topMargin + 2 * (subplotHeight + subplotSpacing),
               graphWidth, subplotHeight);
    
    fill(textColor);
    textAlign(CENTER);
    text("Time (seconds)", width/2, height - 5);
}

void drawSubplot(ArrayList<Float> data, String title, color plotColor,
                float x, float y, float w, float h) {
                  
    // 그래프 영역 그리기
    stroke(100);
    noFill();
    rect(x, y, w, h);
    
    
    fill(textColor);  // 텍스트 검정색 지정
    textAlign(LEFT, TOP);
    text(title, x, y - 15);
    
    // Y축 눈금과 그리드
    for (float value = yMin; value <= yMax; value += 45) {
        float yPos = map(value, yMin, yMax, y + h, y);
        
        // 그리드 선
        stroke(gridColor);
        line(x, yPos, x + w, yPos);
        
        // 눈금 값
        fill(textColor);  // 검정색 텍스트
        textAlign(RIGHT, CENTER);
        text(nf(value, 0, 0) + "°", x - 5, yPos);
    }
    
    // X축 눈금
    for (int i = 0; i < maxDataPoints; i += 40) {
        float xPos = map(i, 0, maxDataPoints, x, x + w);
        
        // 그리드 선
        stroke(gridColor);
        line(xPos, y, xPos, y + h);
        
        // 눈금 값
        if (y + h > height - bottomMargin - 30) {  // 마지막 서브플롯에만 X축 레이블 표시
            fill(textColor);  // 검정색 텍스트
            textAlign(CENTER, TOP);
            text(nf(i/10, 0, 0) + "s", xPos, y + h + 5);
        }
    }
    
    // 데이터 플로팅
    if (data.size() > 1) {
        stroke(plotColor);
        strokeWeight(2);
        noFill();
        
        beginShape();
        for (int i = 0; i < data.size(); i++) {
            float xPos = map(i, 0, maxDataPoints, x, x + w);
            float yPos = map(data.get(i), yMin, yMax, y + h, y);
            vertex(xPos, yPos);
        }
        endShape();
    }
    
    // 현재 값 표시
    if (data.size() > 0) {
        fill(textColor);  // 검정색 텍스트
        textAlign(RIGHT, CENTER);
        text(nf(data.get(data.size()-1), 0, 2) + "°", x + w - 5, y + 15);
    }
}

void draw3D() {
    hint(ENABLE_DEPTH_TEST);
    camera();
    lights();
    
    // 3D 뷰 설정
    translate(width/2, height/2, 0);
    rotateX(cameraRotX);
    rotateY(cameraRotY);
    
    // 좌표축 그리기
    drawAxes(150);
    
    // IMU 모델 그리기
    pushMatrix();
    rotateX(radians(pitch));
    rotateY(radians(yaw));
    rotateZ(radians(-roll));
    drawIMU();
    popMatrix();
    
    // 각도 정보 표시
     camera();
    hint(DISABLE_DEPTH_TEST);
    noLights();
    
    textSize(24);  // 텍스트 크기 증가
    fill(textColor);
    textAlign(LEFT, TOP);
    text("Roll: " + nfp(roll, 0, 1) + "°", 30, 30);
    text("Pitch: " + nfp(pitch, 0, 1) + "°", 30, 60);
    text("Yaw: " + nfp(yaw, 0, 1) + "°", 30, 90);
    
    textSize(18);
}

void drawIMU() {
    // IMU 본체
    fill(200);
    box(400, 80, 240);  // x: 80, y: 240, z: 400 으로 변경
    
    // 전면 표시 (빨간색)
    pushMatrix();
    translate(0, 0, 200);  // z축으로 이동
    fill(255, 0, 0);
    box(80, 80, 20);      // 크기도 조정
    popMatrix();
    
    // 윗면 표시 (초록색)
    pushMatrix();
    translate(0, -120, 0);  // y축으로만 이동
    fill(0, 255, 0);
    box(80, 20, 80);       // 크기 유지
    popMatrix();
}

void drawAxes(float size) {
    strokeWeight(5);  // 선 굵기 증가
    
    // X축 (빨강)
    stroke(255, 0, 0);
    line(0, 0, 0, size*2, 0, 0);  // 크기 2배 증가
    
    // Y축 (초록)
    stroke(0, 255, 0);
    line(0, 0, 0, 0, size*2, 0);
    
    // Z축 (파랑)
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, size*2);
}

void keyPressed() {
    if (key == ' ') {  // 스페이스바로 모드 전환
        is3DMode = !is3DMode;
        println(is3DMode ? "3D 모드로 전환" : "2D 모드로 전환");
    }
}

void serialEvent(Serial port) {
    String input = port.readStringUntil('\n');
    if (input != null) {
        input = input.trim();
        try {
            // 정규 표현식을 사용한 더 유연한 파싱
            String pattern = "Roll: (-?\\d+\\.?\\d*) Pitch: (-?\\d+\\.?\\d*) Yaw: (-?\\d+\\.?\\d*)";
            java.util.regex.Pattern r = java.util.regex.Pattern.compile(pattern);
            java.util.regex.Matcher m = r.matcher(input);
            
            if (m.find()) {
                roll = float(m.group(1));
                pitch = float(m.group(2));
                yaw = float(m.group(3));
                
                rollData.add(roll);
                pitchData.add(pitch);
                yawData.add(yaw);
                
                while (rollData.size() > maxDataPoints) {
                    rollData.remove(0);
                    pitchData.remove(0);
                    yawData.remove(0);
                }
            }
        } catch (Exception e) {
            println("Error parsing data: " + input);
            println("Error message: " + e.getMessage());
        }
    }
}

void dispose() {
    if (myPort != null) {
        myPort.stop();
    }
}
