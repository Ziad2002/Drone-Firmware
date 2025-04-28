#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "sensor.h"
#include <Arduino.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void TaskSensor(void *pvParameters) {
  sensors_event_t accel, mag, gyro, temp;

  while (true) {
    // Read all sensor events
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    // Print Accelerometer data
    Serial.print("[Accel] X: "); Serial.print(accel.acceleration.x);
    Serial.print(" m/s^2, Y: "); Serial.print(accel.acceleration.y);
    Serial.print(" m/s^2, Z: "); Serial.println(accel.acceleration.z);

    // Print Gyroscope data
    Serial.print("[Gyro] X: "); Serial.print(gyro.gyro.x);
    Serial.print(" rad/s, Y: "); Serial.print(gyro.gyro.y);
    Serial.print(" rad/s, Z: "); Serial.println(gyro.gyro.z);

    // // Print Magnetometer data
    Serial.print("[Mag] X: "); Serial.print(mag.magnetic.x);
    Serial.print(" uT, Y: "); Serial.print(mag.magnetic.y);
    Serial.print(" uT, Z: "); Serial.println(mag.magnetic.z);

    // // Optional: Print Temperature data
    // Serial.print("[Temp] "); Serial.print(temp.temperature);
    // Serial.println(" C");

    Serial.println("----------------------------------");

    vTaskDelay(100 / portTICK_PERIOD_MS); // Read every 100 ms
  }
}

void startSensorTask() {
  Serial.println("Initializing LSM9DS1 Sensor...");

  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 sensor! Check wiring.");
    while (1); // Freeze if no sensor found
  }

  setupSensor();

  Serial.println("LSM9DS1 sensor initialized successfully!");

  // Create the Sensor task
  xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 1, NULL, 1);
}
