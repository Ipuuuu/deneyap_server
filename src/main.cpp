
#include <WiFi.h>
#include "esp_http_server.h"
#include <SoftwareSerial.h>
#include <esp_task_wdt.h>

// Use SoftwareSerial for STM32 comms
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

const uint8_t ACCEL_STEP = 12, DECEL_STEP = 15;
bool DEBUG = true;
unsigned long lastClientMillis = 0;  // heartbeat timeout

unsigned long lastSTM32Response = 0;
const unsigned long STM32_TIMEOUT = 5000; // 5 second timeout
bool waitingForSTM32 = false;

// HTTP Request Handlers
esp_err_t handle_setSpeed(httpd_req_t *req);
esp_err_t handle_setServo(httpd_req_t *req);
esp_err_t handle_dumper(httpd_req_t *req);
esp_err_t handle_tof(httpd_req_t *req);
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
void setupPins();
void setupMotorTimer();
void setupWiFi();
void processSTM32Queue();
bool enqueueSTM32Command(const String& cmd);
void flushSTM32Response();

// Timer interrupt - keep minimal
void IRAM_ATTR onMotorTimer() {
  motorUpdateFlag = true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  
  Serial.println("\n=== Robot Control System v3.0 ===");
  
  // Configure watchdog
  esp_task_wdt_init(20, true);
  esp_task_wdt_add(NULL);

  setupPins();
  setupMotorTimer();
  setupWiFi();

  // Initialize STM32 communication - SoftwareSerial at 115200 baud
  stm32Serial.begin(57600); 
  delay(100);
  
  // Clear any startup messages
  flushSTM32Response();
  
  // Initialize queue
  for (int i = 0; i < STM32_QUEUE_SIZE; i++) {
    stm32Queue[i].command = "";
    stm32Queue[i].timestamp = 0;
    stm32Queue[i].sent = false;
  }

  // Start HTTP server
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 16;
  config.task_priority = 5;
  config.stack_size = 4096;
  
  if (httpd_start(&http_server, &config) == ESP_OK) {
    setRoutes();
    Serial.println("HTTP server started on port 80");
  } else {
    Serial.println("Failed to start HTTP server");
  }

  // Initialize STM32 settings
  enqueueSTM32Command("verbose off");
  enqueueSTM32Command("mode coord");
  
  Serial.printf("Ready! Control at http://192.168.4.1/\n");
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
}

void loop() {
  // Reset watchdog early and often
  esp_task_wdt_reset();
  
  // Update motors
  if (motorUpdateFlag) {
    motorUpdateFlag = false;
    updateMotors();
  }
  
  // Process STM32 command queue
  processSTM32Queue();
  
  // Safety timeout
  if (motors.isMoving && (millis() - lastClientMillis > 3000)) {
    gradualStop();
    if (DEBUG) Serial.println("[SAFETY] Client timeout - stopping motors");
  }
  
  // Flush any STM32 responses to prevent buffer overflow
  if (stm32Serial.available() > 32) {
    flushSTM32Response();
  }
  
  // Yield to other tasks
  vTaskDelay(1);
}

void setupPins() {
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(R_EN_LEFT, OUTPUT);
  pinMode(L_EN_LEFT, OUTPUT);
  pinMode(R_EN_RIGHT, OUTPUT);
  pinMode(L_EN_RIGHT, OUTPUT);
  
  // Enable all motor drivers
  digitalWrite(R_EN_LEFT, HIGH);
  digitalWrite(L_EN_LEFT, HIGH);
  digitalWrite(R_EN_RIGHT, HIGH);
  digitalWrite(L_EN_RIGHT, HIGH);
}

void setupMotorTimer() {
  motorTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(motorTimer, &onMotorTimer, true);
  timerAlarmWrite(motorTimer, 25000, true); // 25ms = 40Hz
  timerAlarmEnable(motorTimer);
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  // Configure IP
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  WiFi.softAPConfig(local_IP, gateway, subnet);
  
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
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

void gradualStop() {
  motors.targetLeft = 0;
  motors.targetRight = 0;
}

void emergencyStop() {
  // Immediate stop
  digitalWrite(R_EN_LEFT, LOW);
  digitalWrite(L_EN_LEFT, LOW);
  digitalWrite(R_EN_RIGHT, LOW);
  digitalWrite(L_EN_RIGHT, LOW);
  
  analogWrite(leftForward, 0);
  analogWrite(leftBackward, 0);
  analogWrite(rightForward, 0);   
  analogWrite(rightBackward, 0);

  motors.currentLeft = 0;
  motors.currentRight = 0;
  motors.targetLeft = 0;
  motors.targetRight = 0;
  motors.isMoving = false;
  
  // Re-enable drivers
  delay(100);
  digitalWrite(R_EN_LEFT, HIGH);
  digitalWrite(L_EN_LEFT, HIGH);
  digitalWrite(R_EN_RIGHT, HIGH);
  digitalWrite(L_EN_RIGHT, HIGH);

  if (DEBUG) Serial.println("[EMERGENCY] All motors stopped");
}

// NON-BLOCKING STM32 command queue
bool enqueueSTM32Command(const String& cmd) {
  if (queueCount >= STM32_QUEUE_SIZE) {
    if (DEBUG) Serial.println("[STM32] Queue full, dropping command");
    return false;
  }
  
  stm32Queue[queueTail].command = cmd;
  stm32Queue[queueTail].timestamp = millis();
  stm32Queue[queueTail].sent = false;
  
  queueTail = (queueTail + 1) % STM32_QUEUE_SIZE;
  queueCount++;
  
  return true;
}

void processSTM32Queue() {
  if (queueCount == 0) return;
  
  // Rate limit STM32 commands
  if (millis() - lastSTM32Send < STM32_SEND_INTERVAL) return;
  
  // Send next command
  STM32Command& cmd = stm32Queue[queueHead];
  if (!cmd.sent) {
    stm32Serial.println(cmd.command);
    cmd.sent = true;
    lastSTM32Send = millis();
    
    if (DEBUG) Serial.printf("[STM32] Sent: %s\n", cmd.command.c_str());
  }
  
  // Remove from queue after sending
  queueHead = (queueHead + 1) % STM32_QUEUE_SIZE;
  queueCount--;
}

void flushSTM32Response() {
  while (stm32Serial.available()) {
    stm32Serial.read();
  }
}

// HTTP HANDLERS - All made NON-BLOCKING

esp_err_t handle_setSpeed(httpd_req_t *req) {
  char buf[64];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing body");
  }
  buf[len] = '\0';

  char* space = strchr(buf, ' ');
  if (!space) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Format: 'left right'");
  }
  *space = '\0';

  int left = constrain(atoi(buf), -255, 255);
  int right = constrain(atoi(space + 1), -255, 255);

  motors.targetLeft = left;
  motors.targetRight = right;
  lastClientMillis = millis();

  if (DEBUG) Serial.printf("[SPEED] L=%d R=%d\n", left, right);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_setServo(httpd_req_t *req) {
  char buf[256];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing body");
  }
  buf[len] = '\0';
  
  if (DEBUG) Serial.printf("[SERVO] Body: %s\n", buf);

  const char* servos[] = {"base", "shoulder", "elbow", "wrist", "gripper"};
  const char* ids[] = {"b", "s", "e", "w", "g"};
  int limits[] = {270, 270, 270, 270, 180}; 
  
  char response[256] = "{\"queued\":[";
  int pos = strlen(response);
  bool anyCommand = false;

  // Parse body: param=value&param2=value2
  char* body_copy = strdup(buf);
  char* token = strtok(body_copy, "&");
  
  while (token != NULL) {
    char* equals = strchr(token, '=');
    if (equals) {
      *equals = '\0';
      char* key = token;
      char* value = equals + 1;
      
      for (int i = 0; i < 5; i++) {
        if (strcmp(key, servos[i]) == 0) {
          int angle = constrain(atoi(value), 0, limits[i]);
          
          // FIXED: Queue command instead of blocking
          String cmd = String(ids[i]) + " " + String(angle);
          if (enqueueSTM32Command(cmd)) {
            if (anyCommand) {
              strncat(response, ",", sizeof(response) - strlen(response) - 1);
            }
            char item[64];
            snprintf(item, sizeof(item), "\"%s\":%d", servos[i], angle);
            strncat(response, item, sizeof(response) - strlen(response) - 1);
            anyCommand = true;
            
            if (DEBUG) Serial.printf("[SERVO] Queued: %s=%d\n", servos[i], angle);
          }
          break;
        }
      }
    }
    token = strtok(NULL, "&");
  }
  
  free(body_copy);
  
  if (!anyCommand) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No valid servo parameters");
  }
  
  strncat(response, "]}", sizeof(response) - strlen(response) - 1);
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
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
  // FIXED: Non-blocking TOF read
  enqueueSTM32Command("t");
  
  // Return immediately - don't wait for response
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, "{\"status\":\"requested\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_autoPickup(httpd_req_t *req) {
  enqueueSTM32Command("t a p");
  httpd_resp_send(req, "{\"auto_pickup\":\"queued\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_autoRelease(httpd_req_t *req) {
  enqueueSTM32Command("t a r");
  httpd_resp_send(req, "{\"auto_release\":\"queued\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_emergency(httpd_req_t *req) {
  emergencyStop();
  enqueueSTM32Command("stop");
  httpd_resp_send(req, "{\"emergency\":\"executed\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_health(httpd_req_t *req) {
  httpd_resp_send(req, "{\"status\":\"healthy\"}", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_status(httpd_req_t *req) {
  char status[200];
  snprintf(status, sizeof(status), 
           "{\"device\":\"Deneyap\",\"heap\":%u,\"uptime\":%lu,\"stm32_queue\":%d}",
           ESP.getFreeHeap(), millis(), queueCount);
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, status, HTTPD_RESP_USE_STRLEN);
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
