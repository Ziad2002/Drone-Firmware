#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>

// ===== Wi-Fi and WebSocket config =====
const char* ssid = "Device-Northwestern";
const char* server_host = "18.223.203.35";  // Your Tornado server IP
const uint16_t server_port = 8888;
const char* server_path = "/esp32_image";   // Dedicated image WebSocket endpoint

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

WebSocketsClient webSocket;

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
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

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Camera init failed");
  } else {
    Serial.println("✅ Camera ready");
  }
}

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
    Serial.printf("📤 Sent %d bytes\n", fb->len);
  }

  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n Wi-Fi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  startCamera();

  webSocket.begin(server_host, server_port, server_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

unsigned long lastSend = 0;

void loop() {
  webSocket.loop();

  if (webSocket.isConnected() && millis() - lastSend >= 100) {  // 10 FPS
    sendFrame();
    lastSend = millis();
  }
}
