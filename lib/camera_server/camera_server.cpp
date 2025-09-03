// camera_server.cpp
#include "camera_server.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"

// Camera pins (Deneyap Kart 1A)
#define CAMD2    3
#define CAMD3    4
#define CAMD4    5
#define CAMD5    6
#define CAMD6    7
#define CAMD7    8
#define CAMD8    9
#define CAMD9    10
#define CAMH     11
#define CAMV     12
#define CAMPC    13
#define CAMXC    14
#define CAMSC    15
#define CAMSD    16

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
  config.pin_sccb_sda = CAMSD;
  config.pin_sccb_scl = CAMSC;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);      // 640x480
  s->set_quality(s, 10);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_whitebal(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_colorbar(s, 0);

  Serial.println("Camera initialized");
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

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &fb->buf, &fb->len);
      esp_camera_fb_return(fb);
      if (!jpeg_converted) {
        Serial.println("JPEG conversion failed");
        return ESP_FAIL;
      }
    }

    char buf[64];
    size_t hlen = snprintf(buf, sizeof(buf), STREAM_PART, fb->len);
    httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    httpd_resp_send_chunk(req, buf, hlen);
    httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    if (fb->format != PIXFORMAT_JPEG) {
      free(fb->buf);
    } else {
      esp_camera_fb_return(fb);
    }

    // Frame rate control (~30 FPS)
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  return ESP_OK;
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