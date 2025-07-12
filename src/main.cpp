
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#define LED D13

//wifi credentials
const char * ssid = "CEZERİ-LAB";
const char * password = "cezerilab2024";

// const char * ssid = "God Abeg";
// const char * password = "Godabegooo";

// const char * ssid = "GAMZELICANSU20";
// const char * password = "BOLU5678";


WebServer server(80);


const uint32_t BLINK_INTERVAL = 500;

void handleStatus() ;
void setRoutes();

void setup() {
  Serial.begin(115200);

  //Connect to wifi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected to WiFi. IP Address: ");
  Serial.println(WiFi.localIP());

  setRoutes();

  server.begin();
  Serial.println("server started");
  Serial.println("Available endpoints:");
  Serial.println("  GET  /status     - Get system status");
  Serial.println("  GET  /led     - Post led commands");

}


void loop() {
  server.handleClient();
  
  // Optional: Print periodic status
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 10000) { // Every 10 seconds
    Serial.println("Server running... IP: " + WiFi.localIP().toString());
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
    Serial.println("Received raw JSON: " + state);
    
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
  
}

