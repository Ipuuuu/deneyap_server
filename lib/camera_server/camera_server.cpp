#include "camera_server.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include <esp_task_wdt.h>
#include "Arduino.h"

// Camera pins (CORRECT for Deneyap Kart 1A)
#define CAMD2    19      
#define CAMD3    22     
#define CAMD4    23    
#define CAMD5    21   
#define CAMD6    18     
#define CAMD7    26    
#define CAMD8    35   
#define CAMD9    34  
#define CAMH     39     
#define CAMV     36    
#define CAMPC    5     
#define CAMXC    32     
#define CAMSC    25   
#define CAMSD    33      

// Streaming constants
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
static const char* STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Simple HTML UI
static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Robot Vision</title>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 20px; }
    h1 { color: #2c3e50; }
    img { max-width: 100%; height: auto; border: 2px solid #3498db; border-radius: 8px; }
    .btn { padding: 10px 20px; margin: 10px; font-size: 16px; background: #3498db; color: white; border: none; border-radius: 5px; cursor: pointer; }
    .btn:hover { background: #2980b9; }
    .container { max-width: 800px; margin: 0 auto; }
  </style>
</head>
<body>
  <div class="container">
    <h1>📹 Robot Camera Stream</h1>
    <img src="/stream" alt="Camera Stream" id="streamImg">
    <div>
      <button class="btn" onclick="location.reload()">🔄 Refresh</button>
      <button class="btn" onclick="capturePhoto()">📸 Capture</button>
    </div>
    <p><small>Streaming from Deneyap Kart 1A | ESP32-S3 + OV2640</small></p>
  </div>

  <script>
    function capturePhoto() {
      fetch('/capture')
        .then(res => res.blob())
        .then(blob => {
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = 'robot_' + Date.now() + '.jpg';
          a.click();
          URL.revokeObjectURL(url);
        })
        .catch(err => alert('Capture failed: ' + err));
    }
  </script>
</body>
</html>
)rawliteral";

// Forward declarations
esp_err_t handle_index(httpd_req_t *req);
esp_err_t handle_stream(httpd_req_t *req);
esp_err_t handle_capture(httpd_req_t *req);


bool cameraInit() {
  static bool cameraInitialized = false;
  
  if (cameraInitialized) {
    Serial.println("Camera already initialized");
    return true;
  }

  Serial.println("Starting camera initialization...");
  esp_task_wdt_reset(); // Reset before camera init

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAMD2;
  config.pin_d1 = CAMD3;
  config.pin_d2 = CAMD4;
  config.pin_d3 = CAMD5;
  config.pin_d4 = CAMD6;
  config.pin_d5 = CAMD7;
  config.pin_d6 = CAMD8;
  config.pin_d7 = CAMD9;
  config.pin_xclk = CAMXC;
  config.pin_pclk = CAMPC;
  config.pin_vsync = CAMV;
  config.pin_href = CAMH;
  config.pin_sscb_sda = CAMSD;
  config.pin_sscb_scl = CAMSC;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;  // Start with smaller size
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15;  // Lower quality for less processing
  config.fb_count = 1;       // Single buffer to reduce memory usage

  // Optimize settings based on available PSRAM
  if (psramFound()) {
    Serial.println("PSRAM found - using optimized settings");
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("No PSRAM - using conservative settings");
    config.frame_size = FRAMESIZE_CIF;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 20;
  }

  esp_task_wdt_reset(); // Reset before actual init

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  
  esp_task_wdt_reset(); // Reset after init
  
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  // Get camera sensor for additional configuration
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Failed to get camera sensor");
    return false;
  }
  
  // Apply sensor settings
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 12);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_colorbar(s, 0);
  s->set_whitebal(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);

  esp_task_wdt_reset(); // Final reset after sensor config

  cameraInitialized = true;
  Serial.println("Camera initialized successfully");
  return true;
}

esp_err_t handle_index(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t handle_stream(httpd_req_t *req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  camera_fb_t *fb = NULL; // Declare outside the loop

  while (true) {
    // Check if client is still connected
    if (httpd_req_get_hdr_value_len(req, "Connection") == 0) {
      Serial.println("Client disconnected, ending stream");
      break;
    }

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      res = ESP_FAIL;
      break;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted) {
        Serial.println("JPEG conversion failed");
        res = ESP_FAIL;
        break;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    // Send boundary
    if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK) {
      Serial.println("Failed to send boundary");
      res = ESP_FAIL;
      break;
    }

    // Send headers
    char part_buf[64];
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, _jpg_buf_len);
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
      Serial.println("Failed to send headers");
      res = ESP_FAIL;
      break;
    }

    // Send image data
    if (httpd_resp_send_chunk(req, (const char*)_jpg_buf, _jpg_buf_len) != ESP_OK) {
      Serial.println("Failed to send image data");
      res = ESP_FAIL;
      break;
    }

    // Clean up frame buffer
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    // Essential: Reset watchdog and yield to other tasks
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
  }

  // Cleanup on exit
  if (fb) {
    esp_camera_fb_return(fb);
  }
  if (_jpg_buf) {
    free(_jpg_buf);
  }

  return res;
}

esp_err_t handle_capture(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed: no frame");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  if (fb->format != PIXFORMAT_JPEG) {
    bool converted = frame2jpg(fb, 80, &fb->buf, &fb->len);
    esp_camera_fb_return(fb);
    if (!converted) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    httpd_resp_send(req, (const char*)fb->buf, fb->len);
    free(fb->buf);
  } else {
    httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }

  return ESP_OK;
}

bool startCameraServer(httpd_handle_t *server) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 5;

  if (httpd_start(server, &config) != ESP_OK) {
    Serial.println(" Failed to start camera server");
    return false;
  }

  // Register / endpoint
  {
    httpd_uri_t uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = handle_index,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(*server, &uri);
  }

  // Register /stream endpoint
  {
    httpd_uri_t uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = handle_stream,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(*server, &uri);
  }

  // Register /capture endpoint
  {
    httpd_uri_t uri = {
      .uri = "/capture",
      .method = HTTP_GET,
      .handler = handle_capture,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(*server, &uri);
  }

  Serial.println("Camera server started on port 80");
  return true;
}