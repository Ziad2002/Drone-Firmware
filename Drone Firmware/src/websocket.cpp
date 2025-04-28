#include <WiFi.h>
#include <WebSocketsClient.h>
#include "freertos/queue.h"
#include "websocket.h"
#include <Arduino.h>

// Wi-Fi credentials
const char* ssid = "Device-Northwestern";  
const char* server_host = "18.223.203.35";  
const uint16_t server_port = 8888;

WebSocketsClient webSocket;
extern QueueHandle_t commandQueue;  // Declare the queue from elsewhere (main.cpp)
#define LED 2  // WiFi connection status LED

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  String* cmd = nullptr;
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      digitalWrite(LED, LOW);  // Turn off LED when disconnected
      break;

    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server");
      digitalWrite(LED, HIGH); // Turn on LED when connected
      webSocket.sendTXT("hello from Controller");
      break;

    case WStype_TEXT:
      Serial.printf("[WS] Received: %s\n", payload);
      cmd = new String((char*)payload);

      // Send command to queue
      if (xQueueSend(commandQueue, &cmd, portMAX_DELAY) != pdPASS) {
        Serial.println("[WS] Failed to send command to queue");
        delete cmd; // Free memory if not sent
      }
      break;

    default:
      break;
  }
}

void TaskWebSocket(void *pvParameters) {
  while (true) {
    webSocket.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Non-blocking
  }
}

void startWebSocketTask() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);  // Start LED as OFF

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  webSocket.begin(server_host, server_port, "/esp32_drone");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  xTaskCreatePinnedToCore(TaskWebSocket, "TaskWebSocket", 4096, NULL, 1, NULL, 0);
}
