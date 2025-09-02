// tof_handler.h - Simple production version
#ifndef TOF_HANDLER_H
#define TOF_HANDLER_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <WebServer.h>

struct ToFState {
  uint16_t distance;
  uint8_t status;
  unsigned long timestamp;
  bool autoPickup;
  bool autoRelease;
};

class ToFHandler {
private:
  SoftwareSerial* stm32Serial;
  ToFState state;
  
public:
  ToFHandler(SoftwareSerial* serial) : stm32Serial(serial) {
    state = {0, 0, 0, false, false};
  }
  
  void handleToFGet(WebServer& server) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    // Request fresh reading
    stm32Serial->println("t r");
    
    // Quick wait for response
    unsigned long start = millis();
    while (millis() - start < 100 && !stm32Serial->available()) {
      delay(1);
    }
    
    if (stm32Serial->available()) {
      String response = stm32Serial->readStringUntil('\n');
      parseResponse(response);
    }
    
    // Simple JSON response
    String json = "{\"distance_mm\":" + String(state.distance) + 
                  ",\"distance_cm\":" + String(state.distance/10.0) +
                  ",\"status\":" + String(state.status) +
                  ",\"auto_pickup\":" + String(state.autoPickup ? "true" : "false") +
                  ",\"auto_release\":" + String(state.autoRelease ? "true" : "false") + "}";
    
    server.send(200, "application/json", json);
  }
  
  void handleAutoPickup(WebServer& server) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    stm32Serial->println("t a p");
    state.autoPickup = !state.autoPickup;
    
    server.send(200, "application/json", 
                "{\"auto_pickup\":" + String(state.autoPickup ? "true" : "false") + "}");
  }
  
  void handleAutoRelease(WebServer& server) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    stm32Serial->println("t a r");
    state.autoRelease = !state.autoRelease;
    
    server.send(200, "application/json", 
                "{\"auto_release\":" + String(state.autoRelease ? "true" : "false") + "}");
  }
  
  void handleStatus(WebServer& server) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    stm32Serial->println("t s");
    
    String json = "{\"distance_mm\":" + String(state.distance) + 
                  ",\"status\":" + String(state.status) +
                  ",\"auto_pickup\":" + String(state.autoPickup ? "true" : "false") +
                  ",\"auto_release\":" + String(state.autoRelease ? "true" : "false") +
                  ",\"last_update\":" + String(millis() - state.timestamp) + "}";
    
    server.send(200, "application/json", json);
  }
  
private:
  void parseResponse(String response) {
    response.trim();
    
    if (response.startsWith("TOF:")) {
      // Parse "TOF:1234,0,12345678"
      String data = response.substring(4);
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      
      if (c1 > 0 && c2 > 0) {
        state.distance = data.substring(0, c1).toInt();
        state.status = data.substring(c1 + 1, c2).toInt();
        state.timestamp = millis();
      }
    }
    else if (response.startsWith("AUTO_PICKUP:")) {
      state.autoPickup = response.endsWith("1");
    }
    else if (response.startsWith("AUTO_RELEASE:")) {
      state.autoRelease = response.endsWith("1");
    }
  }
};

#endif