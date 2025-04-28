#include <WiFi.h>
#include <WebSocketsClient.h>

// Wi-Fi credentials
const char* ssid = "Device-Northwestern";  
const char* server_host = "18.223.203.35";  
const uint16_t server_port = 8888;
String msg;
int counter = 0;
#define LED 2

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server");
      webSocket.sendTXT("hello from ESP32");
      break;
    case WStype_TEXT:
      Serial.printf("Received: %s\n", payload);
      msg = (char*)payload;
      // Handle incoming command (e.g., forward, hover)
      Serial.printf("Speed: %d\n", counter);
      if (msg == "up"){
        // counter += 1;
        // analogWrite(LED, counter);
        digitalWrite(LED, HIGH);
      } else if (msg == "down") {
        // counter -= 1;
        // analogWrite(LED, counter);
        digitalWrite(LED, LOW);
      }
      break;
      default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED, OUTPUT);

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

  // Setup WebSocket client
  webSocket.begin(server_host, server_port, "/esp32_drone");  // Tornado route
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);  // try every 5 seconds
}

void loop() {
  webSocket.loop();
}
