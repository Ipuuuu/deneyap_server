// camera_server.h
#pragma once

#include "esp_http_server.h"

/**
 * @brief Start the camera server with all endpoints
 * @param server Pointer to httpd_handle_t (will be assigned)
 * @return true if successful
 */
bool startCameraServer(httpd_handle_t *server);

/**
 * @brief Initialize the camera hardware
 * @return true if camera initialized successfully
 */
bool cameraInit();