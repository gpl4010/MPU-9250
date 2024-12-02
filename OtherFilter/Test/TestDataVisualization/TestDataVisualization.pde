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
float subplotSpacing = 40;

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
    
    drawSubplot(rollData, "", color(255, 0, 0), 
               leftMargin, topMargin, 
               graphWidth, subplotHeight);
               
    drawSubplot(pitchData, "", color(0, 255, 0),
               leftMargin, topMargin + subplotHeight + subplotSpacing,
               graphWidth, subplotHeight);
               
    drawSubplot(yawData, "", color(0, 0, 255),
               leftMargin, topMargin + 2 * (subplotHeight + subplotSpacing),
               graphWidth, subplotHeight);
    
    //각 그래프 이름 중앙 출력
    fill(textColor);
    textAlign(CENTER);
    text("Roll (degrees)", width/2, topMargin - 15);
    text("Pitch (degrees)", width/2, topMargin + subplotHeight + subplotSpacing -15);
    text("Yaw (degrees)", width/2, topMargin + 2 * (subplotHeight + subplotSpacing) - 15);
    
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
    rotateZ(radians(-roll));
    rotateX(radians(pitch));
    rotateY(radians(yaw));
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
    // 차체 색상
    fill(200, 0, 0);  // 빨간색 차체
    noStroke();
    
    // 메인 차체
    pushMatrix();
    translate(0, -20, 0);
    box(300, 60, 140);  // 본체
    
    // 차 지붕
    translate(0, -40, 0);
    box(200, 40, 120);  // 좀 더 작은 지붕
    popMatrix();
    
    // 창문 (검은색)
    fill(30);
    pushMatrix();
    translate(0, -80, 0);
    box(150, 1, 100);  // 윗창문
    popMatrix();
    
    // 바퀴 (검은색)
    fill(30);
    pushMatrix();
    translate(80, 20, 75);  // 앞 오른쪽
    sphere(20);
    popMatrix();
    
    pushMatrix();
    translate(80, 20, -75);  // 앞 왼쪽
    sphere(20);
    popMatrix();
    
    pushMatrix();
    translate(-80, 20, 75);  // 뒤 오른쪽
    sphere(20);
    popMatrix();
    
    pushMatrix();
    translate(-80, 20, -75);  // 뒤 왼쪽
    sphere(20);
    popMatrix();
    
    // 전조등 (노란색)
    fill(255, 255, 0);
    pushMatrix();
    translate(150, -20, 50);
    box(10, 20, 30);
    popMatrix();
    
    pushMatrix();
    translate(150, -20, -50);
    box(10, 20, 30);
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
        float[] values = float(split(input, ','));
        
        if (values.length == 3) {
            // 현재 각도 값 업데이트
            roll = values[0];
            pitch = values[1];
            yaw = values[2];
            
            // 그래프 데이터 업데이트
            rollData.add(roll);
            pitchData.add(pitch);
            yawData.add(yaw);
            
            while (rollData.size() > maxDataPoints) {
                rollData.remove(0);
                pitchData.remove(0);
                yawData.remove(0);
            }
        }
    }
}

void dispose() {
    if (myPort != null) {
        myPort.stop();
    }
}
