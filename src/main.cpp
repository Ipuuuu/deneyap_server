
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "arm.cpp"
#include "arm.h"
#include <ESP32Servo.h>
#define LED D13



//wifi credentials
const char * ssid = "CAR";
const char * password = "cezerilab2024";


// Left side motor pins (Driver 1)

const uint8_t leftForward = A4; // R_PWM_LEFT
const uint8_t leftBackward = A5; // L_PWM_LEFT
const int R_EN_LEFT = A6;    // Left side right enable
const int L_EN_LEFT = A7;    // Left side left enable


// Right side motor pins (Driver 2)
const uint8_t rightForward = A1; //R_PWM_RIGHT
const uint8_t rightBackward = A0; //L_PWM_RIGHT
const int R_EN_RIGHT = A2;  // Right side right enable
const int L_EN_RIGHT = A3;  // Right side left enable

bool DEBUG = true;
const uint8_t DECELERATION_STEP = 8;


WebServer server(80);

struct Control{
  uint8_t leftspeed;
  uint8_t rightspeed;
 
};

struct MotorState {
  int currentLeftSpeed = 0;
  int currentRightSpeed = 0;
  int targetLeftSpeed = 0;
  int targetRightSpeed = 0;
  unsigned long lastUpdateTime = 0;
  bool isMoving = false;
};

struct ArmControl {
  String direction;
};

Control control ; 
RoboticArm roboticArm;
ArmControl armControl;
MotorState motorState;

const uint32_t BLINK_INTERVAL = 200;
const unsigned long UPDATE_INTERVAL = 20; // Update motor speed

void handleStatus() ;
void setRoutes();
void setLeftMotor(int speed);
void setRightMotor(int speed);
void stop();
void estop();
void updateMotorSpeed();

void handleControl();
void handleArm() ;
void handleDumper();
void handleLed();
void serialServo(String, int);


void setup() {
  Serial.begin(115200);

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

  delay(5000);

  //Connect to wifi
  //WiFi.begin(ssid, password);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("Connecting to WiFi");

  setRoutes();

  server.begin();
  Serial.println("server started");
  Serial.println("Available endpoints:");
  Serial.println("  GET  /status     - Get system status");
  Serial.println("  GET  /led     - Post led commands");
  Serial.println("  GET  /control     - Post motor commands");
  Serial.println("  POST /arm        - Post arm commands");
  Serial.println("  GET  /dumper     - Post dumper commands");

  roboticArm.begin(); // Initialize the robotic arm
  motorState.lastUpdateTime = millis(); // Initialize motor state update time


}


void loop() {
  server.handleClient();
  
  //update motor speeds for smooth deceleration
  updateMotorSpeed();
  
  // Optional: Print periodic status
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 15000) { // Every 15 seconds
    Serial.println("Server running..... IP: " + WiFi.localIP().toString());
    lastPrint = millis();
  }

  if(Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim();

    auto servo = command.substring(0, 1);

    int angle = command.substring(1).toInt();
    serialServo(servo, angle);

    Serial.printf("Servo ");
  }
}

void updateMotorSpeed(){
  uint32_t current_time = millis();

  if(current_time - motorState.lastUpdateTime >= UPDATE_INTERVAL){
    bool speedchanged = false;
    
    // update left motor 
    if(motorState.currentLeftSpeed != motorState.targetLeftSpeed){
      int diff = motorState.targetLeftSpeed - motorState.currentLeftSpeed;
      int step = -DECELERATION_STEP;

      if (abs(diff) <= abs(step)) {
        motorState.currentLeftSpeed = motorState.targetLeftSpeed;
      } else {
        motorState.currentLeftSpeed += step;
      }
      speedchanged = true;
    }

    // update right motor 
    if(motorState.currentRightSpeed != motorState.targetRightSpeed){
      int diff = motorState.targetRightSpeed  - motorState.currentRightSpeed;
      int step = -DECELERATION_STEP;

      if (abs(diff) <= abs(step)) {
        motorState.currentRightSpeed = motorState.targetRightSpeed;
      } else {
        motorState.currentRightSpeed += step;
      }
      speedchanged = true;
    }

    if(speedchanged){
      setLeftMotor(motorState.currentLeftSpeed);
      setRightMotor(motorState.currentRightSpeed);
    }
    
    //update moving state
    motorState.isMoving = (motorState.currentLeftSpeed != 0 || motorState.currentRightSpeed != 0);
    motorState.lastUpdateTime = current_time;
  }

}

void setTargetSpeeds(int leftTarget, int rightTarget) {
  // Constrain speeds to valid range
  leftTarget = constrain(leftTarget, -255, 255);
  rightTarget = constrain(rightTarget, -255, 255);
  
  motorState.targetLeftSpeed = leftTarget;
  motorState.targetRightSpeed = rightTarget;
  
  if (DEBUG) {
    Serial.printf("Target speeds set - Left: %d, Right: %d\n", leftTarget, rightTarget);
  }
}

void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  String response = "device=Deneyap Kart RC Truck&version=1.0.0&wifi_rssi=" + String(WiFi.RSSI())
  + "&free_heap=" + String(ESP.getFreeHeap()) + "&uptime_ms=" + String(millis()) +
  "&arm_initialized=" + String(roboticArm.isReady()) +
  "&gripper_state=" + String(roboticArm.getGripperState() ? "open" : "closed");

  server.send(200, "application/x-www-form-urlencoded", response);
  
  if(DEBUG){Serial.println("Status requested");}
}

void handleLed(){  
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (server.hasArg("state")) {
    String stateStr = server.arg("state");
    Serial.println("LED Command received: " + stateStr);
    
    // Convert string to integer
    int ledState = stateStr.toInt();
    
    switch(ledState) {
      case 0:
        digitalWrite(LED, LOW);
        server.send(200, "application/json", "{\"ledStatus\":\"off\"}");
        if(DEBUG){Serial.println("LED turned off");}
        break;
      case 1:
        digitalWrite(LED, HIGH);
        server.send(200, "application/json", "{\"ledStatus\":\"on\"}");
        if(DEBUG){Serial.println("LED turned on");}
        break;
      case 2:{
        uint32_t current_time = millis();
        static uint32_t last_blink_time = 0;
        if(current_time - last_blink_time >= BLINK_INTERVAL) {
          digitalWrite(LED, !digitalRead(LED)); // Toggle LED state
          last_blink_time = current_time;
        }
        server.send(200, "application/json", "{\"ledStatus\":\"blinked once\"}");
        if(DEBUG){Serial.println("LED blinked");}
        break; }
      default:
        server.send(400, "application/json", "{\"error\":\"Unknown LED state\"}");
    }

  }
  else {
    server.send(400, "application/json", "{\"error\":\"Missing 'state' key\"}");
  }
  }


void handleControl(){
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if(server.hasArg("left") && server.hasArg("right")){
    control.leftspeed = server.arg("left").toInt();
    control.rightspeed = server.arg("right").toInt();

    if(DEBUG){Serial.println("Left: " + String(control.leftspeed) + ", Right: " + String(control.rightspeed));}
    setLeftMotor(control.leftspeed);
    setRightMotor(control.rightspeed);
  }
  else{server.send(400, "application/json", "{\"error\":\"Unknown Arg, expecting: left&right \"}");}
  
    
  

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
  server.on("/control", HTTP_POST, handleControl);
  server.on("/arm", HTTP_POST, handleArm); 
  server.on("/dumper", HTTP_POST, handleDumper); 
}

void estop(){
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

void stop(){
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
    estop();
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
    estop();
  }
}


void handleArm() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("Arm command received: " + body);
    
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    if (doc.containsKey("direction")) {
    armControl.direction = doc["direction"].as<String>();
    String response;
      
      if (armControl.direction == "forward") {
        response = roboticArm.pickUp();
      }
      else if (armControl.direction == "backward") {
        response = roboticArm.release();
      }
      else if (armControl.direction == "left") {
        response = roboticArm.scanLeft();
      }
      else if (armControl.direction == "right") {
        response = roboticArm.scanRight();
      }
      else if (armControl.direction == "open") {
        response = roboticArm.open();
      }
      else if (armControl.direction == "close") {
        response = roboticArm.close();
      }
      else if (armControl.direction == "status") {
        response = roboticArm.getStatus();
      }
      else {
        server.send(400, "application/json", "{\"error\":\"Unknown arm command\"}");
        return;
      }
      
      server.send(200, "application/json", response);
    }
    else {
      server.send(400, "application/json", "{\"error\":\"Missing 'command' key\"}");
    }
  }
  else {
    server.send(400, "application/json", "{\"error\":\"Missing JSON body\"}");
  }
}

void handleDumper(){
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("Dumper command received: " + body);
    
    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    if (doc.containsKey("direction")) {
      String action = doc["direction"].as<String>();
      String response;

      if (action == "dumperOpen") {
        response = roboticArm.dumperOpen();
      } 
      else if (action == "dumperClose") {
        response = roboticArm.dumperClose();
      } 
      else {
        server.send(400, "application/json", "{\"error\":\"Unknown dumper action\"}");
        return;
      }
      
      server.send(200, "application/json", response);
    } 
    else {
      server.send(400, "application/json", "{\"error\":\"Missing 'action' key\"}");
    }
  }  
  else {
    server.send(400, "application/json", "{\"error\":\"Missing JSON body\"}");
  }
}

void serialServo(String servo, int angle){
  if (!roboticArm.isReady()) {
    Serial.println("Robotic arm not initialized");
    return;
  }

  if (servo == "b") {
    roboticArm.baseServo.write(angle);
  } else if (servo == "s") {
    roboticArm.shoulderServo.write(angle);
  } else if (servo == "e") {
    roboticArm.elbowServo.write(angle);
  } else if (servo == "w") {
    roboticArm.wristServo.write(angle);
  } else if (servo == "g") {
    roboticArm.gripperServo.write(angle);
  } else if (servo == "d") {
    roboticArm.dumperServo.write(angle);
  } else {
    Serial.println("Unknown servo: " + servo);
  }
}
