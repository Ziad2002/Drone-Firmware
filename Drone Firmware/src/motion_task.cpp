#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "motion_task.h"
#include "control_task.h"
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// === Motor Pins & Channels ===
#define MOTOR1_PIN 12
#define MOTOR2_PIN 13
#define MOTOR3_PIN 4
#define MOTOR4_PIN 2

#define ESC1_CHANNEL 0
#define ESC2_CHANNEL 1
#define ESC3_CHANNEL 2
#define ESC4_CHANNEL 3
#define ESC_FREQ 50
#define ESC_RES_BITS 12


// Sensor and timing variables
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
float accel_offset[3] = {0}, gyro_offset[3] = {0};
float KalmanAngleRoll = 0, KalmanUncertaintyRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyPitch = 4;
float Kalman1DOutput[2] = {0, 0};
unsigned long last_time = 0;
float dt = 0.004;

// === Utility: Convert µs to 12-bit PWM ===
uint32_t usToDuty(int microseconds) {
  return (microseconds * 4095L) / 20000;
}

// === Sensor Calibration ===
void calibrateSensors(int samples) {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  Serial.println("Calibrating sensors... Keep device still.");
  for (int i = 0; i < samples; i++) {
    lsm.read();
    sensors_event_t a, m, g, t;
    lsm.getEvent(&a, &m, &g, &t);
    ax += a.acceleration.x;
    ay += a.acceleration.y;
    az += a.acceleration.z;
    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;
    delay(1);
  }
  accel_offset[0] = ax / samples;
  accel_offset[1] = ay / samples;
  accel_offset[2] = az / samples - 9.81;
  gyro_offset[0] = gx / samples;
  gyro_offset[1] = gy / samples;
  gyro_offset[2] = gz / samples;
  Serial.println("Calibration complete.");
}

// === Kalman Filter ===
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState += dt * KalmanInput;
  KalmanUncertainty += dt * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty *= (1 - KalmanGain);
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// === Setup ESCs and IMU ===
void setupMotionHardware() {
  Wire.begin(32, 33);  // Use GPIO 32 as SDA, GPIO 33 as SCL
  if (!lsm.begin()) {
    Serial.println("Failed to detect LSM9DS1!");
  }

  // Setup ESC channels and attach pins
  ledcSetup(ESC1_CHANNEL, ESC_FREQ, ESC_RES_BITS);
  ledcAttachPin(MOTOR1_PIN, ESC1_CHANNEL);

  ledcSetup(ESC2_CHANNEL, ESC_FREQ, ESC_RES_BITS);
  ledcAttachPin(MOTOR2_PIN, ESC2_CHANNEL);

  ledcSetup(ESC3_CHANNEL, ESC_FREQ, ESC_RES_BITS);
  ledcAttachPin(MOTOR3_PIN, ESC3_CHANNEL);

  ledcSetup(ESC4_CHANNEL, ESC_FREQ, ESC_RES_BITS);
  ledcAttachPin(MOTOR4_PIN, ESC4_CHANNEL);

  // ESC calibration
  Serial.println("Calibrating all ESCs...");
  ledcWrite(ESC1_CHANNEL, usToDuty(2000));
  ledcWrite(ESC2_CHANNEL, usToDuty(2000));
  ledcWrite(ESC3_CHANNEL, usToDuty(2000));
  ledcWrite(ESC4_CHANNEL, usToDuty(2000));
  delay(4000);

  ledcWrite(ESC1_CHANNEL, usToDuty(1000));
  ledcWrite(ESC2_CHANNEL, usToDuty(1000));
  ledcWrite(ESC3_CHANNEL, usToDuty(1000));
  ledcWrite(ESC4_CHANNEL, usToDuty(1000));
  delay(4000);

  Serial.println("ESC Calibration Complete");
}

// === Main Control Loop ===
void motionTask(void *pvParameters) {
  while (true) {
    int localThrottle, localPitch, localRoll, localYaw;

    if (xSemaphoreTake(controlMutex, portMAX_DELAY)) {
      localThrottle = throttle;
      localPitch = pitch;
      localRoll = roll;
      localYaw = yaw;
      xSemaphoreGive(controlMutex);
    }

    uint32_t duty = usToDuty(localThrottle);

    // Print debug info
    // Serial.print("Throttle: ");
    // Serial.print(localThrottle);
    // Serial.print(" -> Duty: ");
    // Serial.println(duty);
 
    // Apply throttle to all motors
    ledcWrite(ESC1_CHANNEL, duty);
    ledcWrite(ESC2_CHANNEL, duty);
    ledcWrite(ESC3_CHANNEL, duty);
    ledcWrite(ESC4_CHANNEL, duty);

    lsm.read();
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    // Δt in seconds
    unsigned long now = micros();
    dt = (now - last_time) / 1000000.0;
    last_time = now;

    // Subtract calibration offsets
    float ax = accel.acceleration.x - accel_offset[0];
    float ay = accel.acceleration.y - accel_offset[1];
    float az = accel.acceleration.z - accel_offset[2];

    float gx = (gyro.gyro.x - gyro_offset[0]) * 180.0 / PI; // Roll rate
    float gy = (gyro.gyro.y - gyro_offset[1]) * 180.0 / PI; // Pitch rate

    float angle_roll_accel = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float angle_pitch_accel = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

    // Kalman filter for roll
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyRoll, gx, angle_roll_accel);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyPitch, gy, angle_pitch_accel);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyPitch = Kalman1DOutput[1];


    Serial.print(KalmanAngleRoll);
    Serial.print("\t");
    Serial.println(KalmanAnglePitch);


    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// === Task Initialization ===
void initMotionTask() {
  setupMotionHardware();
  xTaskCreatePinnedToCore(motionTask, "MotionTask", 8192, NULL, 2, NULL, 0);
}
