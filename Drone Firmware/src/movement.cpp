#include <Arduino.h>       
#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"       
#include "freertos/queue.h"      
#include "movement.h"            

#define backRight 19
#define backLeft 5
#define frontLeft 18
#define frontRight 17

// Access the shared queue created in main.cpp
extern QueueHandle_t commandQueue;

void TaskMovement(void *pvParameters) {
  String* command;

  while (true) {
    if (xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
      Serial.print("[Movement] Executing command: ");
      Serial.println(*command);

      if (*command == "up") {
        analogWrite(backRight, 255);
        analogWrite(backLeft, 255);
        analogWrite(frontLeft,255);
        analogWrite(frontRight,255);
        delay(10000);
        for(int i=255; i >= 0; i--){
          analogWrite(backRight, i);
          analogWrite(backLeft, i);
          analogWrite(frontLeft,i);
          analogWrite(frontRight,i);
        }
        
        Serial.println("[Action] Ascending!");
      }
      else if (*command == "down") {
        
        Serial.println("[Action] Descending!");
      }
      else if (*command == "forward") {
        analogWrite(backRight, 100);
        analogWrite(backLeft, 100);
        analogWrite(frontLeft,60);
        analogWrite(frontRight,60);
        delay(2000);
        for(int i=100; i >= 0; i--){
        analogWrite(backRight, i);
        analogWrite(backLeft, i);
        analogWrite(frontLeft,i);
        analogWrite(frontRight,i);
        }

        Serial.println("[Action] Moving Forward!");
      }
      else if (*command == "backward") {
        analogWrite(backRight, 60);
        analogWrite(backLeft, 60);
        analogWrite(frontLeft,100);
        analogWrite(frontRight,100);
        delay(2000);
        for(int i=100; i >= 0; i--){
        analogWrite(backRight, i);
        analogWrite(backLeft, i);
        analogWrite(frontLeft,i);
        analogWrite(frontRight,i);
        }


        Serial.println("[Action] Moving Backward!");
      }
      else if (*command == "left") {
        analogWrite(backRight, 100);
        analogWrite(backLeft, 60);
        analogWrite(frontLeft, 60);
        analogWrite(frontRight,100);
        delay(2000);
        for(int i=100; i >= 0; i--){
        analogWrite(backRight, i);

        analogWrite(frontRight,i);
        }
        analogWrite(backLeft, 0);
        analogWrite(frontLeft,0);

        Serial.println("[Action] Moving Left!");
      }
      else if (*command == "right") {
        analogWrite(backRight, 60);
        analogWrite(backLeft, 100);
        analogWrite(frontLeft,100);
        analogWrite(frontRight,60);
        delay(2000);
        for(int i=100; i >= 0; i--){
        analogWrite(backRight, i);
        analogWrite(backLeft, i);
        analogWrite(frontLeft,i);
        analogWrite(frontRight,i);
        }
  
        Serial.println("[Action] Moving Right!");
      }
      else if (*command == "hover") {

        Serial.println("[Action] Hovering!");
      }
      else if (*command == "yaw_left") {

        Serial.println("[Action] Rotating Left (Yaw Left)!");
      }
      else if (*command == "yaw_right") {

        Serial.println("[Action] Rotating Right (Yaw Right)!");
      }
      else {
        Serial.println("[Action] Unknown command!");
      }

      delete command;  // Free memory after handling
    }
  }
}

void startMovementTask() {
  Serial.println("Starting Movement Task...");

  xTaskCreatePinnedToCore(TaskMovement, "TaskMovement", 4096, NULL, 1, NULL, 1);
}
