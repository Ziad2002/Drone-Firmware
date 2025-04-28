#include <Arduino.h>
#include "websocket.h"
#include "sensor.h"
#include "movement.h"

// Global command queue
QueueHandle_t commandQueue;

#define motor1 19
#define motor2 18
#define motor3 5
#define motor4 17

void setup() {
  Serial.begin(115200);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  // Create queue before starting tasks
  commandQueue = xQueueCreate(10, sizeof(String*));
  if (commandQueue == NULL) {
    Serial.println("Error creating command queue!");
    while (true);  // Fatal error, freeze
  }

  startWebSocketTask();
  startMovementTask();
  startSensorTask();
}

void loop() {
  // Nothing here - FreeRTOS tasks handle everything
}
