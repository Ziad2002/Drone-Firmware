#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Arduino.h>
#include "control_task.h"
#include <WebSocketsClient.h>
#include <ArduinoJson.h>



WebSocketsClient controlWS;
SemaphoreHandle_t controlMutex = NULL;
int throttle = 1000, pitch = 1500, roll = 1500, yaw = 1500;

void onControlMessage(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
      if (xSemaphoreTake(controlMutex, portMAX_DELAY)) {
        throttle = doc["throttle"];
        pitch = doc["pitch"];
        roll = doc["roll"];
        yaw = doc["yaw"];
        Serial.print("throttle: "); Serial.println(throttle);
        Serial.print("pitch: "); Serial.println(pitch);
        Serial.print("roll: "); Serial.println(roll);
        Serial.print("yaw: "); Serial.println(yaw);
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
