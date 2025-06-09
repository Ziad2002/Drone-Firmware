#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "motion_task.h"
#include "control_task.h"
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <WebSocketsClient.h>

extern volatile int calibrationRequested;
extern bool isRunning;

// Outer loop PID state
float desiredAngleRoll, desiredAnglePitch;
float errorAngleRoll = 0, errorAnglePitch = 0;
float prevErrorAngleRoll = 0, prevErrorAnglePitch = 0;
float iTermAngleRoll = 0, iTermAnglePitch = 0;

// Outer loop PID gains (tune these)
float kpAngleRoll = 0.1, kiAngleRoll = 0, kdAngleRoll = 0.005;
float kpAnglePitch = 0.1, kiAnglePitch = 0, kdAnglePitch = 0.005;


// PID state
float desiredYawRate;
float prevErrRoll = 0, prevErrPitch = 0, prevErrYaw = 0;
float iTermRoll = 0, iTermPitch = 0, iTermYaw = 0;

// PID gains 
float kpRoll = 0.2, kiRoll = 0, kdRoll = 0.05;
float kpPitch = 0.2, kiPitch = 0, kdPitch = 0.05;
float kpYaw = 0.3, kiYaw = 0, kdYaw = 0;

// ESC signal outputs (us)
int motor1, motor2, motor3, motor4;
const int maxSignal = 2000;
const int minSignal = 1000;
const int cutoffSignal = 1000;

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

float computePID(float error, float &prevErr, float &iTerm, float kp, float ki, float kd) {
  float pTerm = kp * error;
  iTerm += ki * (error + prevErr) * dt / 2;
  iTerm = constrain(iTerm, -400, 400);
  float dTerm = kd * (error - prevErr) / dt;
  prevErr = error;
  float output = pTerm + iTerm + dTerm;
  return constrain(output, -400, 400);
}

// === Setup ESCs ===
void setupMotionHardware() {
  Wire.begin(32, 33);  
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

void motionTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4);  // 4ms = 250Hz

  while (true) {

    if (calibrationRequested){

      calibrateSensors(5000);
      // Send message back to server
      extern WebSocketsClient controlWS;
      if (controlWS.isConnected()) {
        controlWS.sendTXT("{\"status\":\"calibration complete\"}");
      }  
      calibrationRequested = 0;
    }

    if (!isRunning) {
      ledcWrite(ESC1_CHANNEL, usToDuty(cutoffSignal));
      ledcWrite(ESC2_CHANNEL, usToDuty(cutoffSignal));
      ledcWrite(ESC3_CHANNEL, usToDuty(cutoffSignal));
      ledcWrite(ESC4_CHANNEL, usToDuty(cutoffSignal));

    }
    else {
      Serial.println("Running");
      // Get control
      int localThrottle, localPitch, localRoll, localYaw;

      if (xSemaphoreTake(controlMutex, portMAX_DELAY)) {
        localThrottle = throttle;
        localPitch = pitch;
        localRoll = roll;
        localYaw = yaw;
        xSemaphoreGive(controlMutex);
      }

      // Calculate desired Rates & Angles
      desiredAngleRoll = 0.10 * (localRoll - 1500); 
      desiredAnglePitch = 0.10 * (localPitch - 1500);


      // Read sensor data
      lsm.read();
      sensors_event_t accel, mag, gyro, temp;
      lsm.getEvent(&accel, &mag, &gyro, &temp);

      // Subtract calibration offsets
      float ax = accel.acceleration.x - accel_offset[0];
      float ay = accel.acceleration.y - accel_offset[1];
      float az = accel.acceleration.z - accel_offset[2];

      float gx = gyro.gyro.x - gyro_offset[0];
      float gy = gyro.gyro.y - gyro_offset[1];
      float gz = gyro.gyro.z - gyro_offset[2];


      // find the rate for pitch, roll and yaw
      float rateRoll = (gx) * 180.0 / PI;
      float ratePitch = (gy) * 180.0 / PI;
      float rateYaw = (gz) * 180.0 / PI;

      // find the angle for pitch and roll
      float angle_pitch_accel = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
      float angle_roll_accel  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;



      kalman_1d(KalmanAngleRoll, KalmanUncertaintyRoll, rateRoll, angle_roll_accel);
      KalmanAngleRoll = Kalman1DOutput[0];
      KalmanUncertaintyRoll = Kalman1DOutput[1];
      
      kalman_1d(KalmanAnglePitch, KalmanUncertaintyPitch, ratePitch, angle_pitch_accel);
      KalmanAnglePitch = Kalman1DOutput[0];
      KalmanUncertaintyPitch = Kalman1DOutput[1];




      // calculate the error between desired rate and rate
      errorAngleRoll = desiredAngleRoll - KalmanAngleRoll;
      errorAnglePitch = desiredAnglePitch - KalmanAnglePitch;

      // Serial.print("Roll: ");
      // Serial.print(KalmanAngleRoll);
      // Serial.print(" | Pitch: ");
      // Serial.print(KalmanAnglePitch);
      // Serial.print(" | ErrRoll: ");
      // Serial.print(errorAngleRoll);
      // Serial.print(" | ErrPitch: ");
      // Serial.println(errorAnglePitch);


      // outer pid loop
      desiredYawRate = 0.15 * (localYaw - 1500);

      // === Apply Deadband to Inner Loop Rate Errors ===


      // inner pid loop


      float desiredRollRate = computePID(errorAngleRoll, prevErrorAngleRoll, iTermAngleRoll,
                                    kpAngleRoll, kiAngleRoll, kdAngleRoll);

      float desiredPitchRate = computePID(errorAnglePitch, prevErrorAnglePitch, iTermAnglePitch,
                                      kpAnglePitch, kiAnglePitch, kdAnglePitch);

      float errRoll = desiredRollRate - rateRoll;
      float errPitch = desiredPitchRate - ratePitch;
      float errYaw = desiredYawRate - rateYaw;

      if (abs(errRoll) < 1.1) errRoll = 0;
      if (abs(errPitch) < 1.1) errPitch = 0;
      if (abs(errYaw) < 1.1) errYaw = 0;

      // Serial.print("Roll: ");
      // Serial.print(rateRoll);
      // Serial.print(" | Pitch: ");
      // Serial.print(ratePitch);
      // Serial.print(" | desired Roll: ");
      // Serial.print(desiredRollRate);
      // Serial.print(" | desired Pitch: ");
      // Serial.println(desiredPitchRate);

      // find pid 
      float pidRoll = computePID(errRoll, prevErrRoll, iTermRoll, kpRoll, kiRoll, kdRoll);
      float pidPitch = computePID(errPitch, prevErrPitch, iTermPitch, kpPitch, kiPitch, kdPitch);
      float pidYaw = computePID(errYaw, prevErrYaw, iTermYaw, kpYaw, kiYaw, kdYaw);

      Serial.print("pidRoll: ");
      Serial.print(pidRoll);
      Serial.print(" | pidPitch: ");
      Serial.print(pidPitch);
      Serial.print(" | pidYaw:");
      Serial.println(pidYaw);
      Serial.print(" | base: ");
      Serial.println(localThrottle);


      // Motors Control
      int base = localThrottle;
      bool pidActive = localThrottle > 1500;

      if (pidActive) {
        motor1 = base - pidRoll - pidPitch - pidYaw;
        motor2 = base - pidRoll + pidPitch + pidYaw;
        motor3 = base + pidRoll + pidPitch - pidYaw;
        motor4 = base + pidRoll - pidPitch + pidYaw;
        
      } else {
        motor1 = motor2 = motor3 = motor4 = base;
      }

      motor1 = constrain(motor1, minSignal, maxSignal);
      motor2 = constrain(motor2, minSignal, maxSignal);
      motor3 = constrain(motor3, minSignal, maxSignal);
      motor4 = constrain(motor4, minSignal, maxSignal);

      if (localThrottle < 1050) {
        motor1 = motor2 = motor3 = motor4 = cutoffSignal;

        // Reset inner loop PID (rate)
        prevErrRoll = prevErrPitch = prevErrYaw = 0;
        iTermRoll = iTermPitch = iTermYaw = 0;

        // Reset outer loop PID (angle)
        prevErrorAngleRoll = prevErrorAnglePitch = 0;
        iTermAngleRoll = iTermAnglePitch = 0;
      }

      // Serial.println(motor1);
      // Serial.println(motor2);
      // Serial.println(motor3);
      // Serial.println(motor4);


      ledcWrite(ESC1_CHANNEL, usToDuty(motor1));
      ledcWrite(ESC2_CHANNEL, usToDuty(motor2));
      ledcWrite(ESC3_CHANNEL, usToDuty(motor3));
      ledcWrite(ESC4_CHANNEL, usToDuty(motor4));

      
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// === Task Initialization ===
void initMotionTask() {
  setupMotionHardware();
  xTaskCreatePinnedToCore(motionTask, "MotionTask", 8192, NULL, 2, NULL, 0);
}
