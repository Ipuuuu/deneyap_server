
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#define LED D13

//wifi credentials
const char * ssid = "CAR";
const char * password = "cezerilab2024";

// const char * ssid = "God Abeg";
// const char * password = "Godabegooo";

// const char * ssid = "GAMZELICANSU20";
// const char * password = "BOLU5678";

// Left side motor pins (Driver 1)

const uint8_t leftForward = A2; // R_PWM_LEFT
const uint8_t leftBackward = A3; // L_PWM_LEFT
const int R_EN_LEFT = A4;    // Left side right enable
const int L_EN_LEFT = A5;    // Left side left enable


// Right side motor pins (Driver 2)
const uint8_t rightForward = A1; //R_PWM_RIGHT
const uint8_t rightBackward = A0; //L_PWM_RIGHT
const int R_EN_RIGHT = A6;  // Right side right enable
const int L_EN_RIGHT = A7;  // Right side left enable


WebServer server(80);

struct Control{
  uint8_t speed;
  String direction;
};

Control control ; 

const uint32_t BLINK_INTERVAL = 200;

void handleStatus() ;
void setRoutes();
void setLeftMotor(int speed);
void setRightMotor(int speed);  
void forward(int speed = 255);
void backward(int speed = 255);
void turnLeft(int speed = 255);
void turnRight(int speed  = 255);
void spinLeft(int speed = 150);
void spinRight(int speed = 150);
void stop();

void handleControl();


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
/*
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  */
 /*
  Serial.println();
  Serial.print("Connected to WiFi. IP Address: ");
  Serial.println(WiFi.localIP());
*/
  setRoutes();

  server.begin();
  Serial.println("server started");
  Serial.println("Available endpoints:");
  Serial.println("  GET  /status     - Get system status");
  Serial.println("  GET  /led     - Post led commands");
  Serial.println("  GET  /control     - Post motor commands");

}


void loop() {
  server.handleClient();
  
  // Optional: Print periodic status
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 10000) { // Every 10 seconds
    Serial.println("Server running..... IP: " + WiFi.localIP().toString());
    lastPrint = millis();
  }
  
  delay(100);
}

void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  // Create JSON response
  StaticJsonDocument<300> doc;
  doc["device"] = "Deneyap Kart RC Truck";
  doc["version"] = "1.0.0";
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["uptime_ms"] = millis();
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
  Serial.println("Status requested");
}

void handleLed(){  
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (server.hasArg("plain")) {
    String state = server.arg("plain");
    Serial.println("LED Command received: " + state);
    
    StaticJsonDocument<100> resdoc;
    DeserializationError error = deserializeJson(resdoc, state);

    if(error){
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    if (resdoc.containsKey("state")){
      uint8_t ledState = resdoc["state"];
      switch(ledState) {
        case 0:
          digitalWrite(LED, LOW);
          server.send(200, "application/json", "{\"status\":\"LED turned off\"}");
          Serial.println("LED turned off");
          break;
        case 1:
          digitalWrite(LED, HIGH);
          server.send(200, "application/json", "{\"status\":\"LED turned on\"}");
          Serial.println("LED turned on");
          break;
        case 2:{
          uint32_t current_time = millis();
          static uint32_t last_blink_time = 0;
          if(current_time - last_blink_time >= BLINK_INTERVAL) {
            digitalWrite(LED, !digitalRead(LED)); // Toggle LED state
            last_blink_time = current_time;
          }
          server.send(200, "application/json", "{\"status\":\"LED blinked once\"}");
          Serial.println("LED blinked once");
          break; }
        default:
          server.send(400, "application/json", "{\"error\":\"Unknown LED state\"}");
      }

    }
    else {
      server.send(400, "application/json", "{\"error\":\"Missing 'state' key\"}");
    }
  }
  else {
    server.send(400, "application/json", "{\"error\":\"Missing JSON body\"}");
  }
}

void handleControl(){
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if(server.hasArg("plain")){
    String body = server.arg("plain");
    Serial.println("Control Commanded received: " + body);
  }

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));

  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  if (doc.containsKey("direction")) {
    control.direction = doc["direction"].as<String>();
    Serial.println("Direction: " + control.direction);
  }

  if (doc.containsKey("speed") ) {
    control.speed = doc["speed"].as<uint8_t>();
    Serial.println("Speed: " + String(control.speed));
  }

  if(control.direction == "forward") {
    forward(control.speed);
    server.send(200, "application/json", "{\"status\":\"Moving forward\"}");
  }
  else if(control.direction == "backward") {
    backward(control.speed);
    server.send(200, "application/json", "{\"status\":\"Moving backward\"}");
  }
  else if(control.direction == "left") {
    turnLeft(control.speed);
    server.send(200, "application/json", "{\"status\":\"Turning left\"}");
  }
  else if(control.direction == "right") {
    turnRight(control.speed);
    server.send(200, "application/json", "{\"status\":\"Turning right\"}");
  }
  else if(control.direction == "stop") {
    stop();
    server.send(200, "application/json", "{\"status\":\"Stopped\"}");
  }
  else if(control.direction == "sleft") {
    spinLeft(control.speed);
    server.send(200, "application/json", "{\"status\":\"Spin Left\"}");
  }
  else if(control.direction == "sright") {
    spinRight(control.speed);
    server.send(200, "application/json", "{\"status\":\"Spin Right\"}");
  }
  else {
    server.send(400, "application/json", "{\"error\":\"Unknown direction\"}");
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

  server.on("/control", HTTP_POST, handleControl);
  
}
void forward(int speed) {
  // Both sides forward at same speed
  setLeftMotor(speed);
  setRightMotor(speed);
}

void backward(int speed) {
  // Both sides backward at same speed
  setLeftMotor(-speed);
  setRightMotor(-speed);
}

void turnLeft(int speed) {
  // Right side faster than left (gentle turn)
  setLeftMotor(speed / 2);
  setRightMotor(speed);
}

void turnRight(int speed) {
  // Left side faster than right (gentle turn)
  setLeftMotor(speed);
  setRightMotor(speed / 2);
}

void spinLeft(int speed) {
  // Left side backward, right side forward (pivot turn)
  setLeftMotor(-speed);
  setRightMotor(speed);
}

void spinRight(int speed) {
  // Left side forward, right side backward (pivot turn)
  setLeftMotor(speed);
  setRightMotor(-speed);
}

void stop(){
  // Disable all drivers for complete stop
  digitalWrite(R_EN_LEFT, LOW);
  digitalWrite(L_EN_LEFT, LOW);
  digitalWrite(R_EN_RIGHT, LOW);
  digitalWrite(L_EN_RIGHT, LOW);
  
  // Also set PWM to 0
  setLeftMotor(0);
  setRightMotor(0);

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