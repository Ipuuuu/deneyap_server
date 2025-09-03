
#include <WiFi.h>
#include <WebServer.h>
#include <SoftwareSerial.h>

SoftwareSerial stm32Serial(D14, D13); // RX, TX pins for STM32 communication

// wifi credentials
const char * ssid = "CAR";
const char * password = "cezerilab2024";

// //PINS FOR THE DENEYAP KART 1A V2
// // Left side motor pins (Driver 1)
// const uint8_t leftForward = A4; // R_PWM_LEFT
// const uint8_t leftBackward = A5; // L_PWM_LEFT
// const int R_EN_LEFT = A6;    // Left side right enable
// const int L_EN_LEFT = A7;    // Left side left enable

// // Right side motor pins (Driver 2)
// const uint8_t rightForward = A1; //R_PWM_RIGHT
// const uint8_t rightBackward = A0; //L_PWM_RIGHT
// const int R_EN_RIGHT = A2;  // Right side right enable
// const int L_EN_RIGHT = A3;  // Right side left enable


//PINS FOR THE DENEYAP KART 1A 
// Left side motor pins (Driver 1)
const uint8_t leftForward = A4; // R_PWM_LEFT blue
const uint8_t leftBackward = A5; // L_PWM_LEFT purple
const int R_EN_LEFT = D7;    // Left side right enable grey
const int L_EN_LEFT = D9;    // Left side left enable white

// Right side motor pins (Driver 2)
const uint8_t rightForward = D0; //R_PWM_RIGHT yellow
const uint8_t rightBackward = D1; //L_PWM_RIGHT green
const int R_EN_RIGHT = D4;  // Right side right enable orange
const int L_EN_RIGHT = D6;  // Right side left enable red

unsigned long lastClientMillis = 0;  // heartbeat timeout
// Motor state
struct MotorState {
  int currentLeft = 0, currentRight = 0;
  int targetLeft = 0, targetRight = 0;
  bool isMoving = false;
} motors;

// Config
const uint8_t ACCEL_STEP = 12, DECEL_STEP = 8;
bool DEBUG = true;
volatile bool motorUpdateFlag = false;
hw_timer_t *timer = NULL;


// Global Objects
WebServer server(80);

// Handlers
void handleServo() ;
void handleControl();
void handleDumper();
void handleToF();
void handleAutoPickup();
void handleAutoRelease();
void handleEmergencyStop();

// Functions declarations
void setRoutes();
void setMotor(uint8_t forwardPin, uint8_t backwardPin, int speed);
void gradualStop();
void emergencyStop();
void updateMotors();

// Utility Functions
void setupPins();
void setupTimer();
void setupWiFi();


// timer for motor update
void IRAM_ATTR onTimer() {
  motorUpdateFlag = true;  
}

void setup() {
  Serial.begin(115200);
  stm32Serial.begin(115200);

  setupPins();
  setupTimer();
  setupWiFi();
  setRoutes();

  server.begin();
  Serial.println("server started");

  if (DEBUG) {
    Serial.println("=== DEBUG MODE ENABLED ===");
    Serial.println("Available endpoints:");
    Serial.println("  POST /setSpeed - Motor control");
    Serial.println("  POST /emergency  - Emergency stop (immediate)");
    Serial.println("  POST /setServo - Servo control");  
    Serial.println("  POST /dumper   - Dumper control");
    Serial.println("  GET  /tof      - Distance reading");
    Serial.println("========================");
  }

}

void loop() {
  server.handleClient();
  lastClientMillis = millis(); 

  if(motorUpdateFlag){
    motorUpdateFlag = false;
    updateMotors();
  }

  // Check for inactivity (e.g., > 3.5 seconds since last request)
  if (motors.isMoving && (millis() - lastClientMillis > 3500)) {
    gradualStop();  // Smooth stop if no contact
    if (DEBUG) {
      Serial.println("[SAFETY] No client activity for 2s - gradual stop triggered");
    }
  }
 
}

void setupPins(){

  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(R_EN_LEFT, OUTPUT);
  pinMode(L_EN_LEFT, OUTPUT);
  pinMode(R_EN_RIGHT, OUTPUT);
  pinMode(L_EN_RIGHT, OUTPUT);
  
  // Enable all motor driver sides
  digitalWrite(R_EN_LEFT, HIGH);
  digitalWrite(L_EN_LEFT, HIGH);
  digitalWrite(R_EN_RIGHT, HIGH);
  digitalWrite(L_EN_RIGHT, HIGH);

}

void setupTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 30000, true); // 30ms
  timerAlarmEnable(timer);
}

void setupWiFi() {
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}



void updateMotors() {
  bool changed = false;
  
  // Update left motor
  if (motors.currentLeft != motors.targetLeft) {
    int diff = motors.targetLeft - motors.currentLeft;
    int step = (diff > 0) ? ACCEL_STEP : -DECEL_STEP;
    
    if (abs(diff) <= abs(step)) {
      motors.currentLeft = motors.targetLeft;
    } else {
      motors.currentLeft += step;
    }

    
    changed = true;
  }
  
  // Update right motor
  if (motors.currentRight != motors.targetRight) {
    int diff = motors.targetRight - motors.currentRight;
    int step = (diff > 0) ? ACCEL_STEP : -DECEL_STEP;
    
    if (abs(diff) <= abs(step)) {
      motors.currentRight = motors.targetRight;
    } else {
      motors.currentRight += step;
    }
    changed = true;
  }
  
  if (changed) {
    setMotor(leftForward, leftBackward, motors.currentLeft);
    setMotor(rightForward, rightBackward, motors.currentRight);
    motors.isMoving = (motors.currentLeft != 0 || motors.currentRight != 0);
  }
}


void handleControl() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"Missing body\"}");
    return;
  }

  String body = server.arg("plain");
  int space = body.indexOf(' ');
  if (space <= 0) {
    server.send(400, "application/json", "{\"error\":\"Format: 'left right'\"}");
    return;
  }

  int left = constrain(body.substring(0, space).toInt(), -255, 255);
  int right = constrain(body.substring(space + 1).toInt(), -255, 255);
  
  motors.targetLeft = left;
  motors.targetRight = right;
  
  server.send(200, "application/json", 
              "{\"left\":" + String(left) + ",\"right\":" + String(right) + "}");
}

void handleServo() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  String response = "{\"updated\":{";
  bool moved = false;
  
  String servos[] = {"base", "shoulder", "elbow", "wrist", "gripper"};
  String ids[] = {"b", "s", "e", "w", "g"};
  int limits[] = {270, 270, 270, 270, 180};
  
  for (int i = 0; i < 5; i++) {
    if (server.hasArg(servos[i])) {
      int angle = constrain(server.arg(servos[i]).toInt(), 0, limits[i]);
      stm32Serial.println(ids[i] + " " + String(angle));

      if (DEBUG) {
        Serial.printf("[SERVO] %s -> %d degrees (cmd: %s %d)\n", 
                    servos[i].c_str(), angle, ids[i].c_str(), angle);
      }

      response += "\"" + servos[i] + "\":" + String(angle) + ",";
      moved = true;
    }
  }
  
  if (moved) {
    response.remove(response.length() - 1);
    response += "}}";
    server.send(200, "application/json", response);

    if (DEBUG) {
      Serial.println("[SERVO] Commands sent to STM32");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"No servo specified\"}");
  }
}

void setRoutes(){
  server.onNotFound([]() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Endpoint not found"); });

  server.on("/health", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  });

  server.on("/status", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String response = "device=Deneyap&free_heap=" + String(ESP.getFreeHeap()) + 
                     "&uptime=" + String(millis());
    server.send(200, "text/plain", response);
  });

  server.on("/setSpeed", HTTP_POST, handleControl);
  server.on("/setServo", HTTP_POST, handleServo);
  server.on("/dumper", HTTP_POST, handleDumper);
  server.on("/tof", HTTP_GET, handleToF);
  server.on("/tof/auto/pickup", HTTP_POST, handleAutoPickup);
  server.on("/tof/auto/release", HTTP_POST, handleAutoRelease);
  server.on("/emergency", HTTP_POST, handleEmergencyStop);
}

void emergencyStop(){
  if (DEBUG) {
    Serial.println("[EMERGENCY] Emergency stop activated!");
  }

  // Disable all drivers for complete stop
  digitalWrite(R_EN_LEFT, LOW);
  digitalWrite(L_EN_LEFT, LOW);
  digitalWrite(R_EN_RIGHT, LOW);
  digitalWrite(L_EN_RIGHT, LOW);
  
  analogWrite(leftForward, 0);
  analogWrite(leftBackward, 0);
  analogWrite(rightForward, 0);   
  analogWrite(rightBackward, 0);

  uint32_t current_time = millis();
  static uint32_t last_stop_time = 0;
  if(current_time - last_stop_time >= 10) {
    // Re-enable for next operation
    digitalWrite(R_EN_LEFT, HIGH);
    digitalWrite(L_EN_LEFT, HIGH);
    digitalWrite(R_EN_RIGHT, HIGH);
    digitalWrite(L_EN_RIGHT, HIGH);
    last_stop_time = current_time;
  }
}

void gradualStop(){
  motors.targetLeft = 0;
  motors.targetRight = 0;
}

void handleEmergencyStop() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  // Reset motor states first
  motors.currentLeft = 0;
  motors.currentRight = 0;
  motors.targetLeft = 0;
  motors.targetRight = 0;
  motors.isMoving = false;
  
  emergencyStop();
  
  server.send(200, "application/json", "{\"emergency\":\"stopped\"}");
}

void setMotor(uint8_t forwardPin, uint8_t backwardPin, int speed) {
  if (speed > 0) {
    analogWrite(forwardPin, speed);
    analogWrite(backwardPin, 0);
  } else if (speed < 0) {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, -speed);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, 0);
  }
}


void handleDumper() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (!server.hasArg("state")) {
    server.send(400, "application/json", "{\"error\":\"Missing state\"}");
    return;
  }
  
  int state = server.arg("state").toInt();
  if (state == 0) {
    stm32Serial.println("d 0");
    server.send(200, "application/json", "{\"dumper\":\"closed\"}");
    if (DEBUG) {
      Serial.println("[DUMPER] Closed (0 degrees)");
    }
  } else if (state == 1) {
    stm32Serial.println("d 90");
    server.send(200, "application/json", "{\"dumper\":\"opened\"}");
    if (DEBUG) {
      Serial.println("[DUMPER] Opened (90 degrees)");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"State: 0 or 1\"}");
  }
}

void handleToF() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  stm32Serial.println("t");
  
  // Wait for response
  unsigned long start = millis();
  while (millis() - start < 100 && !stm32Serial.available()) {
    delay(1);
  }
  
  String distance = "8190"; // Default
  if (stm32Serial.available()) {
    distance = stm32Serial.readStringUntil('\n');
    distance.trim();
    
    if (DEBUG) {
      Serial.printf("[TOF] Received: %s mm (%.1f cm)\n", 
                    distance.c_str(), distance.toFloat()/10.0);
    }
  } 
  else if (DEBUG) {
    Serial.println("[TOF] No response from STM32, using default");
  }
  
  String json = "{\"distance_mm\":" + distance + 
                ",\"distance_cm\":" + String(distance.toFloat()/10.0) + "}";
  
  server.send(200, "application/json", json);
}

void handleAutoPickup() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  stm32Serial.println("t a p");
  server.send(200, "application/json", "{\"auto_pickup\":\"toggled\"}");
}

void handleAutoRelease() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  stm32Serial.println("t a r");
  server.send(200, "application/json", "{\"auto_release\":\"toggled\"}");
}

