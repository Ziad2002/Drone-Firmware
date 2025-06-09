#include "camera_task.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



const char* server_host = "18.223.203.35";
const uint16_t server_port = 8888;
const char* server_path = "/esp32_image";

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("WebSocket connected");
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket disconnected");
  }
}

void sendFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  if (webSocket.isConnected()) {
    webSocket.sendBIN(fb->buf, fb->len);
    //Serial.printf("Sent %d bytes", fb->len);
  }

  esp_camera_fb_return(fb);
}

void cameraTask(void *pvParameters) {
  while (true) {
    webSocket.loop();
    if (webSocket.isConnected()) {
      sendFrame();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // ~10 FPS
  }
}

void initCameraTask() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - Camera task will not start");
    return;
  }
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;  // More stable than 20 MHz
  config.pixel_format = PIXFORMAT_JPEG;
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  config.fb_location = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed with error 0x%x\n", err);
  } else {
    Serial.println("✅ Camera initialized");
  }

  webSocket.begin(server_host, server_port, server_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  xTaskCreatePinnedToCore(cameraTask, "CameraTask", 8192, NULL, 1, NULL, 1);
}
