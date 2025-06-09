#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Arduino.h>
#include "control_task.h"
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

WebSocketsClient controlWS;
SemaphoreHandle_t controlMutex = NULL;
bool isRunning = false;
int calibrationRequested = 0;
extern void calibrateSensors(int samples); 

int throttle = 1000, pitch = 1500, roll = 1500, yaw = 1500;

void onControlMessage(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.print("Received: ");
    Serial.println((char*)payload);

    if (strcmp((char*)payload, "START") == 0) {
      isRunning = true;
      calibrationRequested = 1;
      Serial.println("Drone STARTED");
      return;
    }
    if (strcmp((char*)payload, "STOP") == 0) {
      isRunning = false;
      calibrationRequested = 0;
      Serial.println("Drone STOPPED");
      return;
    }

    StaticJsonDocument<200> doc;
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {

      // Handle control values only if drone is running
      if (xSemaphoreTake(controlMutex, portMAX_DELAY)) {
        throttle = doc["throttle"];
        pitch = doc["pitch"];
        roll = doc["roll"];
        yaw = doc["yaw"];

        xSemaphoreGive(controlMutex);
      }
      }
    }
  }


void controlTask(void *pvParameters) {
  while (true) {
    controlWS.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void initControlTask() {
  controlMutex = xSemaphoreCreateMutex();
  controlWS.begin("18.223.203.35", 8888, "/esp32_drone");
  controlWS.onEvent(onControlMessage);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 1, NULL, 1);
}
