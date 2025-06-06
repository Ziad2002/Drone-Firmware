#include <Arduino.h>
#include <WiFi.h>
#include "camera_task.h"
#include "control_task.h"
#include "motion_task.h"

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin("Device-Northwestern");
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  initCameraTask();
  initControlTask();
  initMotionTask();
}

void loop() {

}