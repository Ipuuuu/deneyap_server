
#include <WiFi.h>
#include "esp_http_server.h"
#include <SoftwareSerial.h>
#include <esp_task_wdt.h> // Watchdog timer

SoftwareSerial stm32Serial(D14, D13); // RX, TX pins for STM32 communication

// wifi credentials
const char * ssid = "CAR";
const char * password = "cezerilab2024";

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

// Motor state
struct MotorState {
  int currentLeft = 0, currentRight = 0;
  int targetLeft = 0, targetRight = 0;
  bool isMoving = false;
} motors;

// Config
httpd_handle_t http_server = NULL;
hw_timer_t *motorTimer = NULL;
volatile bool motorUpdateFlag = false;

const uint8_t ACCEL_STEP = 12, DECEL_STEP = 8;
bool DEBUG = true;
unsigned long lastClientMillis = 0;  // heartbeat timeout

unsigned long lastSTM32Response = 0;
const unsigned long STM32_TIMEOUT = 5000; // 5 second timeout
bool waitingForSTM32 = false;

// HTTP Request Handlers
esp_err_t handle_setSpeed(httpd_req_t *req);
esp_err_t handle_setServo(httpd_req_t *req);
esp_err_t handle_dumper(httpd_req_t *req);
esp_err_t handle_gripper(httpd_req_t *req);
esp_err_t handle_tof(httpd_req_t *req); 
// esp_err_t handle_fixedBack(httpd_req_t *req); 
esp_err_t handle_autoPickup(httpd_req_t *req);
esp_err_t handle_autoRelease(httpd_req_t *req);
esp_err_t handle_emergency(httpd_req_t *req);
esp_err_t handle_health(httpd_req_t *req);
esp_err_t handle_status(httpd_req_t *req);

// Functions declarations
void setRoutes();
void setMotor(uint8_t forwardPin, uint8_t backwardPin, int speed);
void gradualStop();
void emergencyStop();
void updateMotors();

// Utility Functions
void setupPins();
void setupMotorTimer();
void setupWiFi();
char* get_query_param(httpd_req_t *req, const char* key);
bool waitForSTM32Response(unsigned long timeout_ms);
bool sendCoordinatedMovement(const char* servos[], const char* ids[], 
                           int angles[], int count, int limits[]);

// timer for motor update
void IRAM_ATTR onMotorTimer() {
  motorUpdateFlag = true;
}


void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println("\n=== Starting Robot Control System ===");
  Serial.println("Reset reason: " + String(esp_reset_reason()));

  // Disable existing GPIO ISR service to prevent conflicts
  gpio_uninstall_isr_service();
  
  // Initialize watchdog with longer timeout
  esp_task_wdt_init(15, true);  // 15-second timeout
  esp_task_wdt_add(NULL);
  esp_task_wdt_reset();

  Serial.println("Setting up pins...");
  setupPins();
  esp_task_wdt_reset();

  Serial.println("Setting up motor timer...");
  setupMotorTimer();
  esp_task_wdt_reset();

  Serial.println("Setting up WiFi...");
  setupWiFi();
  esp_task_wdt_reset();

  Serial.println("Initializing STM32 communication...");
  stm32Serial.begin(115200);
  esp_task_wdt_reset();

  // Start HTTP server for robot control
  Serial.println("Starting robot control server...");
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 16;
  if (httpd_start(&http_server, &config) == ESP_OK) {
    setRoutes();
    Serial.println("Robot control server started on port 80");
  } else {
    Serial.println("Failed to start robot control server");
  }
  esp_task_wdt_reset();

  if (DEBUG) {
    Serial.println("\n=== Debug Mode Active ===");
    Serial.println("Robot Control Endpoints:");
    Serial.println("POST: /setSpeed, /setServo, /dumper, /auto/pickup, /auto/release, /emergency");
    Serial.println("GET:  /tof, /health, /status");
    Serial.printf("Robot Control: http://%s/\n", WiFi.softAPIP().toString().c_str());
    Serial.println("Camera will be available at: http://192.168.4.200/");
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM available: %s\n", psramFound() ? "Yes" : "No");
  }

  Serial.println("=== Setup Complete - Starting Main Loop ===\n");
  esp_task_wdt_reset();
}

// void setup() {
//   Serial.begin(115200);
//   stm32Serial.begin(115200);
//   delay(1000);

//   gpio_uninstall_isr_service();
//   Serial.println("Reset reason: " + String(esp_reset_reason()));

//   // Initialize watchdog
//   esp_task_wdt_init(15, true);  // 15-second timeout
//   esp_task_wdt_add(NULL);       // Add current thread

//   setupPins();
//   setupMotorTimer();
//   setupWiFi();


//   bool cameraReady = cameraInit();
//   if (cameraReady) {
//     if (startCameraServer(&http_server)) {
//       setRoutes();
//       if (DEBUG) Serial.println("Camera server and routes initialized");
//     }
//   } else {
//     Serial.println("Camera not available - starting HTTP server without camera endpoints");
//     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//     config.server_port = 80;
//     if (httpd_start(&http_server, &config) == ESP_OK) {
//       setRoutes();
//     }
//   }

//   if (DEBUG) {
//     Serial.println("Debug Mode Active — Endpoints:");
//     Serial.println("  POST /setSpeed, /setServo, /dumper, /auto/pickup, /auto/release, /emergency");
//     Serial.println("  GET /tof, /health, /status, /stream, /capture");
//   }

// }

void loop() {
  
  lastClientMillis = millis(); 

  if(motorUpdateFlag){
    motorUpdateFlag = false;
    updateMotors();
  }

  // CRITICAL: Reset watchdog more frequently
  esp_task_wdt_reset();
  
  // Force yield to prevent blocking
  vTaskDelay(1); // This gives other tasks a chance to run
  
  // Check for inactivity
  if (motors.isMoving && (millis() - lastClientMillis > 3500)) {
    gradualStop();
    if (DEBUG) {
      Serial.println("[SAFETY] No client activity for 3.5s - gradual stop triggered");
    }
  }
  
  // Another watchdog reset at the end
  esp_task_wdt_reset();
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

void setupMotorTimer() {
  motorTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(motorTimer, &onMotorTimer, true);
  timerAlarmWrite(motorTimer, 30000, true); // 30ms
  timerAlarmEnable(motorTimer);
}

void setupWiFi() {
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void setMotor(uint8_t forwardPin, uint8_t backwardPin, int speed) {
  // Ensure motor drivers are ENABLED before applying PWM
  digitalWrite(R_EN_LEFT, HIGH);
  digitalWrite(L_EN_LEFT, HIGH);
  digitalWrite(R_EN_RIGHT, HIGH);
  digitalWrite(L_EN_RIGHT, HIGH);

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

  esp_task_wdt_reset(); // Reset watchdog timer
}

void gradualStop(){
  motors.targetLeft = 0;
  motors.targetRight = 0;
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

  // Reset motor states 
  motors.currentLeft = 0;
  motors.currentRight = 0;
  motors.targetLeft = 0;
  motors.targetRight = 0;
  motors.isMoving = false;

  if (DEBUG) Serial.println("EMERGENCY STOP");
}

//HANDLER DEFINITIONS
esp_err_t handle_setSpeed(httpd_req_t *req) {
  char buf[64];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing request body");
  }
  buf[len] = '\0';

  // Find space delimiter
  char* space = strchr(buf, ' ');
  if (!space) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Format: 'left right'");
  }
  *space = '\0';  // Terminate first number

  // Convert to integers
  int left = constrain(atoi(buf), -255, 255);
  int right = constrain(atoi(space + 1), -255, 255);

  motors.targetLeft = left;
  motors.targetRight = right;
  lastClientMillis = millis();

  if (DEBUG) Serial.printf("[SPEED] L=%d R=%d\n", left, right);

  char resp[100];
  snprintf(resp, sizeof(resp), "{\"left\":%d,\"right\":%d}", left, right);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_setServo(httpd_req_t *req) {
  // Read the request body
  char buf[256];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing request body - Expected format: 'param=value&param2=value2'");
  }
  buf[len] = '\0';
  
  if (DEBUG) Serial.printf("[SERVO] Body: %s\n", buf);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  const char* servos[] = {"base", "shoulder", "elbow", "wrist", "gripper"};
  const char* ids[] = {"b", "s", "e", "w", "g"};
  int limits[] = {270, 270, 270, 270, 180}; // come back!!!
  
  // Arrays to store the movement data
  int angles[5];
  bool servoMoved[5] = {false};
  int moveCount = 0;
  
  char response[256] = "{\"updated\":{";
  int pos = strlen(response);

  // Parse the body data (format: param=value&param2=value2)
  char* body_copy = strdup(buf);
  char* token = strtok(body_copy, "&");
  
  while (token != NULL) {
    // Split token into key=value
    char* equals = strchr(token, '=');
    if (equals) {
      *equals = '\0';
      char* key = token;
      char* value = equals + 1;
      
      // Check if this key matches any servo
      for (int i = 0; i < 5; i++) {
        if (strcmp(key, servos[i]) == 0) {
          int angle = constrain(atoi(value), 0, limits[i]);
          angles[i] = angle;
          servoMoved[i] = true;
          moveCount++;
          
          // Add to response JSON
          int n = snprintf(response + pos, sizeof(response) - pos, 
                         "\"%s\":%d,", servos[i], angle);
          if (n > 0 && n < (int)(sizeof(response) - pos)) {
            pos += n;
          }
          break;
        }
      }
    }
    token = strtok(NULL, "&");
  }
  
  free(body_copy);

  if (moveCount == 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, 
                              "No valid servo parameters found. Format: 'base=90&shoulder=45'");
  }
  
  // Send coordinated movement to STM32
  const char* moveServos[5];
  const char* moveIds[5];
  int moveAngles[5];
  int moveLimits[5];
  int actualMoveCount = 0;
  
  for (int i = 0; i < 5; i++) {
    if (servoMoved[i]) {
      moveServos[actualMoveCount] = servos[i];
      moveIds[actualMoveCount] = ids[i];
      moveAngles[actualMoveCount] = angles[i];
      moveLimits[actualMoveCount] = limits[i];
      actualMoveCount++;
    }
  }
  
  // Clear any pending STM32 serial data
  while (stm32Serial.available()) {
    stm32Serial.read();
  }
  
  // Send the coordinated movement
  bool success = sendCoordinatedMovement(moveServos, moveIds, moveAngles, actualMoveCount, moveLimits);
  
  if (!success) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, 
                              "STM32 movement failed or timed out");
  }
  
  // Complete the JSON response
  if (pos > 11 && pos < (int)sizeof(response) - 1) {
    response[pos-1] = '}';
    response[pos] = '}';
    response[pos+1] = '\0';
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  
  return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Response build error");
}


esp_err_t handle_dumper(httpd_req_t *req) {
  // Read request body
  char buf[64];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing request body - Expected: 'state=0' or 'state=1'");
  }
  buf[len] = '\0';
  
  // Parse state=X format
  char* equals = strchr(buf, '=');
  if (!equals) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid format - Expected: 'state=0' or 'state=1'");
  }
  
  char* key = buf;
  char* value = equals + 1;
  *equals = '\0';
  
  // Check if key is "state"
  if (strcmp(key, "state") != 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameter - Expected: 'state=0' or 'state=1'");
  }

  int state = atoi(value);
  
  if (DEBUG) Serial.printf("[DUMPER] State=%d\n", state);

  if (state == 0) {
    stm32Serial.println("d 0");
    httpd_resp_send(req, "{\"dumper\":\"opened\"}", HTTPD_RESP_USE_STRLEN);
  } else if (state == 1) {
    stm32Serial.println("d 90");
    httpd_resp_send(req, "{\"dumper\":\"closed\"}", HTTPD_RESP_USE_STRLEN);
  } else {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "state must be 0 or 1");
  }
  return ESP_OK;
}

esp_err_t handle_gripper(httpd_req_t *req) {
  // Read request body
  char buf[64];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing request body - Expected: 'state=0' or 'state=1'");
  }
  buf[len] = '\0';
  
  // Parse state=X format
  char* equals = strchr(buf, '=');
  if (!equals) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid format - Expected: 'state=0' or 'state=1'");
  }
  
  char* key = buf;
  char* value = equals + 1;
  *equals = '\0';
  
  // Check if key is "state"
  if (strcmp(key, "state") != 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameter - Expected: 'state=0' or 'state=1'");
  }

  int state = atoi(value);
  
  if (DEBUG) Serial.printf("[DUMPER] State=%d\n", state);

  if (state == 0) {
    stm32Serial.println("g 0");
    httpd_resp_send(req, "{\"gripper\":\"opened\"}", HTTPD_RESP_USE_STRLEN);
  } else if (state == 1) {
    stm32Serial.println("d 40");
    httpd_resp_send(req, "{\"gripper\":\"closed\"}", HTTPD_RESP_USE_STRLEN);
  } else {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "state must be 0 or 1");
  }
  return ESP_OK;
}


esp_err_t handle_tof(httpd_req_t *req) {
  stm32Serial.println("t");
  unsigned long start = millis();
  
  while (millis() - start < 100 && !stm32Serial.available()) {
    delay(1);
  }

  int distance = 8190;  // Default in mm
  if (stm32Serial.available()) {
    String distStr = stm32Serial.readStringUntil('\n');
    distStr.trim();
    
    // Validate and convert
    if (distStr.length() > 0 && distStr[0] >= '0' && distStr[0] <= '9') {
      distance = distStr.toInt();
    }
  }

  if (DEBUG) Serial.printf("[TOF] Distance=%dmm\n", distance);

  // Build response safely
  char json[100];
  snprintf(json, sizeof(json), 
           "{\"distance_mm\":%d,\"distance_cm\":%.1f}", 
           distance, distance / 10.0f);
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_autoPickup(httpd_req_t *req) {
  stm32Serial.println("t a p");
  httpd_resp_send(req, "{\"auto_pickup\":\"started\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_autoRelease(httpd_req_t *req) {
  stm32Serial.println("t a r");
  httpd_resp_send(req, "{\"auto_release\":\"started\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_emergency(httpd_req_t *req) {
  emergencyStop();
  httpd_resp_send(req, "{\"emergency\":\"stopped\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_health(httpd_req_t *req) {
  httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_status(httpd_req_t *req) {
  String resp = "device=Deneyap&free_heap=" + String(ESP.getFreeHeap()) + "&uptime=" + String(millis());
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

void setRoutes() {
  #define ON(PATH, METHOD, HANDLER) \
  [&]() { \
    httpd_uri_t uri = { \
      .uri = PATH, \
      .method = METHOD, \
      .handler = HANDLER, \
      .user_ctx = NULL \
    }; \
    httpd_register_uri_handler(http_server, &uri); \
  }()

  ON("/setSpeed", HTTP_POST, handle_setSpeed);
  ON("/setServo", HTTP_POST, handle_setServo);
  ON("/dumper", HTTP_POST, handle_dumper);
  ON("/gripper", HTTP_POST, handle_gripper);
  // ON("/fixedBack", HTTP_POST, handle_fixedBack);
  ON("/tof", HTTP_GET, handle_tof);
  ON("/auto/pickup", HTTP_POST, handle_autoPickup);
  ON("/auto/release", HTTP_POST, handle_autoRelease);
  ON("/emergency", HTTP_POST, handle_emergency);
  ON("/health", HTTP_GET, handle_health);
  ON("/status", HTTP_GET, handle_status);

  #undef ON
}

// Helper function to wait for STM32 response
bool waitForSTM32Response(unsigned long timeout_ms = 2000) {
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout_ms) {
    if (stm32Serial.available()) {
      char c = stm32Serial.read();
      if (c == '\n') {
        response.trim();
        if (DEBUG) Serial.printf("[STM32] Response: %s\n", response.c_str());
        
        // Check for completion responses
        if (response.startsWith("DONE:") || 
            response.startsWith("QUEUED:") || 
            response.startsWith("EXEC:")) {
          return true;
        }
        response = "";
      } else {
        response += c;
      }
    }
    delay(1); // Small delay to prevent tight loop
  }
  
  if (DEBUG) Serial.println("[STM32] Timeout waiting for response");
  return false;
}

// Helper function to send coordinated movement
bool sendCoordinatedMovement(const char* servos[], const char* ids[], 
                           int angles[], int count, int limits[]) {
  // Send all servo commands as a batch
  for (int i = 0; i < count; i++) {
    stm32Serial.printf("%c %d\n", ids[i][0], angles[i]);
    if (DEBUG) Serial.printf("[STM32] Sent: %c %d\n", ids[i][0], angles[i]);
    
    // Small delay between commands to ensure proper queuing
    delay(10);
  }
  
  // Wait for the last movement to complete
  // The STM32 will send DONE: messages for each servo when it completes
  int completedCount = 0;
  unsigned long start = millis();
  String response = "";
  
  while (completedCount < count && millis() - start < 10000) { // 10 second total timeout
    if (stm32Serial.available()) {
      char c = stm32Serial.read();
      if (c == '\n') {
        response.trim();
        if (DEBUG) Serial.printf("[STM32] Response: %s\n", response.c_str());
        
        if (response.startsWith("DONE:")) {
          completedCount++;
        } else if (response.startsWith("QUEUE_FULL")) {
          if (DEBUG) Serial.println("[STM32] Queue full, movement failed");
          return false;
        }
        response = "";
      } else {
        response += c;
      }
    }
    delay(1);
  }
  
  return completedCount == count;
}
