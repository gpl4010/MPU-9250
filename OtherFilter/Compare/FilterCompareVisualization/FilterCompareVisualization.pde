import processing.serial.*;

Serial myPort;

// 데이터 저장 ArrayList (9개의 데이터)
ArrayList<Float> MyFilter_Roll = new ArrayList<Float>();
ArrayList<Float> Rawdata_Roll = new ArrayList<Float>();
ArrayList<Float> KalmanFilter_Roll = new ArrayList<Float>();
ArrayList<Float> MyFilter_Pitch = new ArrayList<Float>();
ArrayList<Float> Rawdata_Pitch = new ArrayList<Float>();
ArrayList<Float> KalmanFilter_Pitch = new ArrayList<Float>();
ArrayList<Float> MyFilter_Yaw = new ArrayList<Float>();
ArrayList<Float> Rawdata_Yaw = new ArrayList<Float>();
ArrayList<Float> KalmanFilter_Yaw = new ArrayList<Float>();

// 현재 표시 모드 (1, 2, 3)
int displayMode = 1;

// 그래프 설정
int maxDataPoints = 200;
float yMin = -180;  // 필요에 따라 조정
float yMax = 180;   // 필요에 따라 조정
float leftMargin = 60;
float rightMargin = 30;
float topMargin = 30;
float bottomMargin = 30;
float subplotSpacing = 40;

// 색상 설정
color textColor = color(0);
color gridColor = color(200);
color backgroundColor = color(240);

void setup() {
    size(1200, 800);
    
    // 시리얼 포트 설정
    String[] ports = Serial.list();
    println("Available serial ports:");
    for (int i = 0; i < ports.length; i++) {
        println(i + ": " + ports[i]);
    }
    
    try {
        myPort = new Serial(this, Serial.list()[2], 115200);
        myPort.bufferUntil('\n');
        println("Port connected: " + Serial.list()[2]);
    } catch (Exception e) {
        println("Port connection failed!");
        println("Error: " + e.getMessage());
        exit();
    }
    
    textSize(18);
}

void draw() {
    background(backgroundColor);
    
    float subplotHeight = (height - topMargin - bottomMargin - 2 * subplotSpacing) / 3;
    float graphWidth = width - leftMargin - rightMargin;
    
    // 현재 모드에 따라 다른 데이터 그룹 표시
    switch(displayMode) {
        case 1: //roll
            drawSubplot(MyFilter_Roll, "MyFilter_Roll", color(255, 0, 0), 
                       leftMargin, topMargin, graphWidth, subplotHeight);
            drawSubplot(Rawdata_Roll, "Rawdata_Roll", color(0, 255, 0),
                       leftMargin, topMargin + subplotHeight + subplotSpacing, 
                       graphWidth, subplotHeight);
            drawSubplot(KalmanFilter_Roll, "KalmanFilter_Roll", color(0, 0, 255),
                       leftMargin, topMargin + 2 * (subplotHeight + subplotSpacing), 
                       graphWidth, subplotHeight);
            break;
            
        case 2: //pitch
            drawSubplot(MyFilter_Pitch, "MyFilter_Pitch", color(255, 0, 0),
                       leftMargin, topMargin, graphWidth, subplotHeight);
            drawSubplot(Rawdata_Pitch, "Rawdata_Pitch", color(0, 255, 0),
                       leftMargin, topMargin + subplotHeight + subplotSpacing, 
                       graphWidth, subplotHeight);
            drawSubplot(KalmanFilter_Pitch, "KalmanFilter_Pitch", color(0, 0, 255),
                       leftMargin, topMargin + 2 * (subplotHeight + subplotSpacing), 
                       graphWidth, subplotHeight);
            break;
            
        case 3: //yaw
            drawSubplot(MyFilter_Yaw, "MyFilter_Yaw", color(255, 0, 0),
                       leftMargin, topMargin, graphWidth, subplotHeight);
            drawSubplot(Rawdata_Yaw, "Rawdata_Yaw", color(0, 255, 0),
                       leftMargin, topMargin + subplotHeight + subplotSpacing, 
                       graphWidth, subplotHeight);
            drawSubplot(KalmanFilter_Yaw, "KalmanFilter_Yaw", color(0, 0, 255),
                       leftMargin, topMargin + 2 * (subplotHeight + subplotSpacing), 
                       graphWidth, subplotHeight);
            break;
    }
    
    // 현재 모드 표시
    fill(textColor);
    textAlign(LEFT, BOTTOM);
    text("Current Mode: " + displayMode + " (Press 1/2/3 to change)", 10, height - 10);
}

void keyPressed() {
    // 1, 2, 3 키로 모드 변경
    if (key == '1') displayMode = 1;
    if (key == '2') displayMode = 2;
    if (key == '3') displayMode = 3;
}

void serialEvent(Serial port) {
    String input = port.readStringUntil('\n');
    if (input != null) {
        input = input.trim();
        String[] values = split(input, ',');
        
        if (values.length == 9) {
            try {
                // 9개의 데이터 파싱
                float a = float(values[0]);
                float b = float(values[1]);
                float c = float(values[2]);
                float d = float(values[3]);
                float e = float(values[4]);
                float f = float(values[5]);
                float g = float(values[6]);
                float h = float(values[7]);
                float i = float(values[8]);
                
                // 데이터 저장
                adMyFilter_Pitch(MyFilter_Roll, a);
                adMyFilter_Pitch(Rawdata_Roll, d);
                adMyFilter_Pitch(KalmanFilter_Roll, g);
                adMyFilter_Pitch(MyFilter_Pitch, b);
                adMyFilter_Pitch(Rawdata_Pitch, e);
                adMyFilter_Pitch(KalmanFilter_Pitch, h);
                adMyFilter_Pitch(MyFilter_Yaw, c);
                adMyFilter_Pitch(Rawdata_Yaw, f);
                adMyFilter_Pitch(KalmanFilter_Yaw, i);
                
            } catch (Exception e) {
                println("Error parsing data: " + input);
                println("Error message: " + e.getMessage());
            }
        }
    }
}

void adMyFilter_Pitch(ArrayList<Float> dataList, float value) {
    dataList.add(value);
    while (dataList.size() > maxDataPoints) {
        dataList.remove(0);
    }
}

void drawSubplot(ArrayList<Float> data, String title, color plotColor,
                float x, float y, float w, float h) {
    // 그래프 영역
    stroke(100);
    noFill();
    rect(x, y, w, h);
    
    // 제목
    fill(textColor);
    textAlign(CENTER, TOP);
    text(title, width/2, y - 15);
    
    // Y축 눈금과 그리드
    for (float value = yMin; value <= yMax; value += 45) {
        float yPos = map(value, yMin, yMax, y + h, y);
        
        stroke(gridColor);
        line(x, yPos, x + w, yPos);
        
        fill(textColor);
        textAlign(RIGHT, CENTER);
        text(nf(value, 0, 0), x - 5, yPos);
    }
    
    // X축 눈금
    for (int i = 0; i < maxDataPoints; i += 40) {
        float xPos = map(i, 0, maxDataPoints, x, x + w);
        stroke(gridColor);
        line(xPos, y, xPos, y + h);
        
        if (y + h > height - bottomMargin - 30) {
            fill(textColor);
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
        fill(textColor);
        textAlign(LEFT, CENTER);
        text(nf(data.get(data.size()-1), 0, 2), x + 5, y + 15);
    }
}

void dispose() {
    if (myPort != null) {
        myPort.stop();
    }
}
