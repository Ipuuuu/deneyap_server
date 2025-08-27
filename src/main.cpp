
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <ESP32Servo.h>

#define LED D13

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

// Config constants
bool DEBUG = true;
volatile bool motorUpdateFlag = false;
hw_timer_t *timer = NULL;
const uint8_t DECELERATION_STEP = 8;
const uint8_t ACCELERATION_STEP = 12;
const uint32_t BLINK_INTERVAL = 200;
const unsigned long UPDATE_INTERVAL = 20; // Update motor speed
const unsigned long STATUS_PRINT_INTERVAL = 15000; // Print status every 15 seconds

struct Control{
  uint8_t leftspeed;
  uint8_t rightspeed;
};

struct MotorState {
  int currentLeftSpeed = 0;
  int currentRightSpeed = 0;
  int targetLeftSpeed = 0;
  int targetRightSpeed = 0;
  // unsigned long lastUpdateTime = 0;|
  bool isMoving = false;
};



// Global Objects

WebServer server(80);

Control control ; 
MotorState motorState;

// Handlers
void handleStatus() ;
void handleControl();
void handleArm() ;
void handleDumper();
void handleLed();

// Functions declarations
void setRoutes();
void setLeftMotor(int speed);
void setRightMotor(int speed);
void setTargetSpeeds(int leftTarget, int rightTarget);
void gradualStop();
void emergencyStop();
void updateMotorSpeed();

// Utility Functions
void sendServoCommand(String, int);
void setupPins();
void setupWiFi();
void printAvailableEndpoints();
void printPeriodicStatus();

// timer for motor update
void IRAM_ATTR onTimer() {
  motorUpdateFlag = true;  
}

void setup() {
  Serial.begin(115200);
  stm32Serial.begin(115200);
  setupPins();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 30000, true); //30ms interval
  timerAlarmEnable(timer);

  setupWiFi();
  setRoutes();

  server.begin();
  Serial.println("server started");
  printAvailableEndpoints();

  
  // motorState.lastUpdateTime = millis(); // Initialize motor state update time

}


void loop() {
  server.handleClient();
  if(motorUpdateFlag){
    motorUpdateFlag = false;
    updateMotorSpeed();
  }
  printPeriodicStatus();
}

void setupPins(){
  pinMode(LED, OUTPUT);

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

void setupWiFi() {
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void printAvailableEndpoints() {
  Serial.println("Available endpoints:");
  Serial.println("  GET  /health     - Health check");
  Serial.println("  GET  /status     - Get system status");
  Serial.println("  POST /setSpeed    - Motor control (body: 'left right')");
  Serial.println("  POST /setServo   - Servo control (body: base=60&shoulder=90...)");
  Serial.println("  POST /dumper     - Dumper commands (JSON)");
  Serial.println("  POST /led        - LED control");
}

void printPeriodicStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > STATUS_PRINT_INTERVAL) {
    Serial.println("Server running... AP IP: " + WiFi.softAPIP().toString());
    Serial.printf("Motors - Left: %d, Right: %d, Moving: %s\n", 
                  motorState.currentLeftSpeed, 
                  motorState.currentRightSpeed, 
                  motorState.isMoving ? "Yes" : "No");
    lastPrint = millis();
  }
}

void updateMotorSpeed(){
  
  bool speedchanged = false;
  
  // Debug: Print current state before updates
  // if (DEBUG) {
  //   Serial.printf("[MOTOR DEBUG] Current: L=%d R=%d | Target: L=%d R=%d\n", 
  //                 motorState.currentLeftSpeed, motorState.currentRightSpeed,
  //                 motorState.targetLeftSpeed, motorState.targetRightSpeed);
  // }
  
  // update left motor 
  if(motorState.currentLeftSpeed != motorState.targetLeftSpeed){
    int diff = motorState.targetLeftSpeed - motorState.currentLeftSpeed;
    int step = (abs(diff) > 0) ? ((diff > 0) ? ACCELERATION_STEP : -DECELERATION_STEP) : 0;

    int oldLeftSpeed = motorState.currentLeftSpeed;
    
    if (abs(diff) <= abs(step)) {
      motorState.currentLeftSpeed = motorState.targetLeftSpeed;
    }
    else {
      motorState.currentLeftSpeed += step;
    }
    
    if (DEBUG) {
      Serial.printf("[LEFT] %d -> %d (step: %d, diff: %d)\n", 
                    oldLeftSpeed, motorState.currentLeftSpeed, step, diff);
    }
    
    speedchanged = true;
  }

  // update right motor 
  if(motorState.currentRightSpeed != motorState.targetRightSpeed){
    int diff = motorState.targetRightSpeed - motorState.currentRightSpeed;
    int step = (abs(diff) > 0) ? ((diff > 0) ? ACCELERATION_STEP : -DECELERATION_STEP) : 0;

    int oldRightSpeed = motorState.currentRightSpeed;
    
    if (abs(diff) <= abs(step)) {
      motorState.currentRightSpeed = motorState.targetRightSpeed;
    } 
    else {
      motorState.currentRightSpeed += step;
    }
    
    if (DEBUG) {
      Serial.printf("[RIGHT] %d -> %d (step: %d, diff: %d)\n", 
                    oldRightSpeed, motorState.currentRightSpeed, step, diff);
    }
    
    speedchanged = true;
  }

  if(speedchanged){
    if (DEBUG) {
      Serial.printf("[APPLYING] Setting motors to L=%d R=%d\n", 
                    motorState.currentLeftSpeed, motorState.currentRightSpeed);
    }
    
    setLeftMotor(motorState.currentLeftSpeed);
    setRightMotor(motorState.currentRightSpeed);
    
    if (DEBUG) {
      Serial.println("[APPLIED] Motor values written to hardware");
    }
    }
    
    //update moving state
    bool wasMoving = motorState.isMoving;
    motorState.isMoving = (motorState.currentLeftSpeed != 0 || motorState.currentRightSpeed != 0);
    
    if (DEBUG && wasMoving != motorState.isMoving) {
      Serial.printf("[STATE CHANGE] Moving: %s -> %s\n", 
                    wasMoving ? "true" : "false", 
                    motorState.isMoving ? "true" : "false");
    }
    
  
}

void setTargetSpeeds(int leftTarget, int rightTarget) {
  // Constrain speeds to valid range
  leftTarget = constrain(leftTarget, -255, 255);
  rightTarget = constrain(rightTarget, -255, 255);
  
  // Only print if targets actually changed
  if (DEBUG && (leftTarget != motorState.targetLeftSpeed || rightTarget != motorState.targetRightSpeed)) {
    Serial.printf("[NEW TARGETS] Left: %d->%d, Right: %d->%d\n", 
                  motorState.targetLeftSpeed, leftTarget,
                  motorState.targetRightSpeed, rightTarget);
  }
  
  motorState.targetLeftSpeed = leftTarget;
  motorState.targetRightSpeed = rightTarget;
}

void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  String response = "device=Deneyap Kart RC Truck&version=1.0.0&wifi_rssi=" + String(WiFi.RSSI())
  + "&free_heap=" + String(ESP.getFreeHeap()) + "&uptime_ms=" + String(millis());

  server.send(200, "application/x-www-form-urlencoded", response);
  
  if(DEBUG){Serial.println("Status requested");}
}

void handleLed(){  
  server.sendHeader("Access-Control-Allow-Origin", "*");

  }


void handleControl(){
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    // Parse "255 255" format (left right)
    int spaceIndex = body.indexOf(' ');
    if (spaceIndex > 0) {
      int leftSpeed = body.substring(0, spaceIndex).toInt();
      int rightSpeed = body.substring(spaceIndex + 1).toInt();
      
      if (DEBUG) {
        Serial.println("Control - Left: " + String(leftSpeed) + ", Right: " + String(rightSpeed));
      }
      
      setTargetSpeeds(leftSpeed, rightSpeed);
      server.send(200, "application/json", "{\"status\":\"ok\",\"left\":" + String(leftSpeed) + ",\"right\":" + String(rightSpeed) + "}");
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid format, expecting: 'left right' (e.g., '255 255')\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing body\"}");
  }

}

void handleSetServo() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  bool servoMoved = false;
  String response = "{\"servos_updated\":{";
  
  // Check each possible servo argument independently
  if (server.hasArg("base")) {
    int angle = constrain(server.arg("base").toInt(), 0, 270);
    sendServoCommand("b", angle);
    response += "\"base\":" + String(angle) + ",";
    servoMoved = true;
  }
  
  if (server.hasArg("shoulder")) {
    int angle = constrain(server.arg("shoulder").toInt(), 0, 270);
    sendServoCommand("s", angle);
    response += "\"shoulder\":" + String(angle) + ",";
    servoMoved = true;
  }
  
  if (server.hasArg("elbow")) {
    int angle = constrain(server.arg("elbow").toInt(), 0, 270);
    sendServoCommand("e", angle);
    response += "\"elbow\":" + String(angle) + ",";
    servoMoved = true;
  }
  
  if (server.hasArg("wrist")) {
    int angle = constrain(server.arg("wrist").toInt(), 0, 270);
    sendServoCommand("w", angle);
    response += "\"wrist\":" + String(angle) + ",";
    servoMoved = true;
  }
  
  if (server.hasArg("gripper")) {
    int angle = constrain(server.arg("gripper").toInt(), 0, 270);
    sendServoCommand("g", angle);
    response += "\"gripper\":" + String(angle) + ",";
    servoMoved = true;
  }
  
  if (servoMoved) {
    response.remove(response.length() - 1); // Remove last comma
    response += "}}";
    server.send(200, "application/json", response);
    
    if (DEBUG) {
      Serial.println("Servo command processed and sent to STM32");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"No valid servo arguments provided. Use: base, shoulder, elbow, wrist, or gripper\"}");
  }
}

void setRoutes(){
  server.onNotFound([]() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Endpoint not found"); });

  server.on("/health", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"status\":\"ok\",\"uptime\":" + String(millis()) + "}");
  });

  server.on("/status", HTTP_GET, handleStatus);
  server.on("/led", HTTP_POST, handleLed);
  server.on("/setSpeed", HTTP_POST, handleControl);
  server.on("/setServo", HTTP_POST, handleSetServo); 
  server.on("/dumper", HTTP_POST, handleDumper); 
}

void emergencyStop(){
  // Disable all drivers for complete stop
  digitalWrite(R_EN_LEFT, LOW);
  digitalWrite(L_EN_LEFT, LOW);
  digitalWrite(R_EN_RIGHT, LOW);
  digitalWrite(L_EN_RIGHT, LOW);
  
  analogWrite(leftForward, 0);
  analogWrite(leftBackward, 0);

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
  // Set target speeds to 0 for gradual deceleration (smooth stop)
  setTargetSpeeds(0, 0);
}


void setLeftMotor(int speed) {
  // Speed range: -255 (full reverse) to +255 (full forward)
  if (speed > 0) {
    // Forward direction
    analogWrite(leftForward, speed);
    analogWrite(leftBackward, 0);
  } 
  else if (speed < 0) {
    // Reverse direction
    analogWrite(leftForward, 0);
    analogWrite(leftBackward, -speed);
  } 
  else {
    // Stop
    analogWrite(leftForward, 0);
    analogWrite(leftBackward, 0);
  }
}

void setRightMotor(int speed) {
  // Speed range: -255 (full reverse) to +255 (full forward)
  if (speed > 0) {
    // Forward direction
    analogWrite(rightForward, speed);
    analogWrite(rightBackward, 0);
  } 
  else if (speed < 0) {
    // Reverse direction
    analogWrite(rightForward, 0);
    analogWrite(rightBackward, -speed);
  } 
  else {
    // Stop
    analogWrite(rightForward, 0);
    analogWrite(rightBackward, 0);
  }
}

void handleDumper() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (server.hasArg("state")) {
    int state = server.arg("state").toInt();
    
    if (state == 0) {
      // Close dumper - 0 degrees
      sendServoCommand("d", 0);
      server.send(200, "application/json", "{\"dumper\":\"closed\",\"angle\":0}");
      if (DEBUG) {
        Serial.println("Dumper closed (0 degrees)");
      }
    } 
    else if (state == 1) {
      // Open dumper - 90 degrees
      sendServoCommand("d", 90);
      server.send(200, "application/json", "{\"dumper\":\"opened\",\"angle\":90}");
      if (DEBUG) {
        Serial.println("Dumper opened (90 degrees)");
      }
    } 
    else {
      server.send(400, "application/json", "{\"error\":\"Invalid state. Use 0 for close or 1 for open\"}");
    }
  } 
  else {
    server.send(400, "application/json", "{\"error\":\"Missing 'state' parameter. Use state=0 for close or state=1 for open\"}");
  }
}

void sendServoCommand(String servo, int angle) {
  String command = servo + " " + String(angle);
  stm32Serial.println(command);
  
  if (DEBUG) {
    Serial.println("Sent to STM32: " + command);
  }
  
  // Optional: Wait for acknowledgment from STM32
  // You can add timeout logic here if needed
  unsigned long timeout = millis() + 1000; // 1 second timeout
  while (millis() < timeout && !stm32Serial.available()) {
    delay(1);
  }
  
  if (stm32Serial.available()) {
    String response = stm32Serial.readStringUntil('\n');
    if (DEBUG) {
      Serial.println("STM32 Response: " + response);
    }
  }
}
