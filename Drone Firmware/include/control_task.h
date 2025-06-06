#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t controlMutex;
extern int throttle, pitch, roll, yaw;
void initControlTask();
#endif