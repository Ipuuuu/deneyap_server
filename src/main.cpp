
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

//motor pins
const uint8_t rightForward = A0;
const uint8_t rightBackward = A1;
const uint8_t leftForward = A2;
const uint8_t leftBackward = A3;

const uint8_t leftSpeed = D14;
const uint8_t RightSpeed = D13;

WebServer server(80);

struct Control{
  uint8_t left_speed;
  uint8_t right_speed;
  String direction;
};

Control control ; 

const uint32_t BLINK_INTERVAL = 200;

void handleStatus() ;
void setRoutes();
void back_and_forth(uint8_t l = 200, uint8_t r = 200);
void forward(uint8_t l = 200, uint8_t r = 200);
void backward(uint8_t l = 200, uint8_t r = 200);
void stop();
void turn_left(uint8_t l = 150, uint8_t r = 255);
void turn_right(uint8_t l = 255, uint8_t r = 150);
void handleControl();


void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);

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

  if (doc.containsKey("right_speed") && doc.containsKey("left_speed")) {
    control.left_speed = doc["left_speed"].as<uint8_t>();
    control.right_speed = doc["right_speed"].as<uint8_t>();
    Serial.println("Left Speed: " + String(control.left_speed));
    Serial.println("Right Speed: " + String(control.right_speed));
  }

  if(control.direction == "forward") {
    forward(control.left_speed, control.right_speed);
    server.send(200, "application/json", "{\"status\":\"Moving forward\"}");
  }
  else if(control.direction == "backward") {
    backward(control.left_speed, control.right_speed);
    server.send(200, "application/json", "{\"status\":\"Moving backward\"}");
  }
  else if(control.direction == "left") {
    turn_left(control.left_speed, control.right_speed);
    server.send(200, "application/json", "{\"status\":\"Turning left\"}");
  }
  else if(control.direction == "right") {
    turn_right(control.left_speed, control.right_speed);
    server.send(200, "application/json", "{\"status\":\"Turning right\"}");
  }
  else if(control.direction == "stop") {
    stop();
    server.send(200, "application/json", "{\"status\":\"Stopped\"}");
  }
  else if(control.direction == "back_and_forth") {
    back_and_forth(control.left_speed, control.right_speed);
    server.send(200, "application/json", "{\"status\":\"Back and forth\"}");
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


void back_and_forth(uint8_t l, uint8_t r){
  analogWrite(leftSpeed, l);
  analogWrite(RightSpeed, r);

  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);

  delay(2000);

  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);

  delay(2000);

}

void forward(uint8_t l, uint8_t r){
  analogWrite(leftSpeed, l);
  analogWrite(RightSpeed, r);

  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);

}

void backward(uint8_t l , uint8_t r ){
  analogWrite(leftSpeed, l);
  analogWrite(RightSpeed, r);

  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
}

void stop(){
  analogWrite(leftSpeed, 0);
  analogWrite(RightSpeed, 0);

  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
}

void turn_left(uint8_t l, uint8_t r){
  analogWrite(leftSpeed, l);
  analogWrite(RightSpeed, r); 

  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
}

void turn_right(uint8_t l, uint8_t r){
  analogWrite(leftSpeed, l);
  analogWrite(RightSpeed, r);

  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
}
