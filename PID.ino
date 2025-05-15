#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define backRight 19
#define backLeft 5
#define frontLeft 18
#define frontRight 17


Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Calibration offsets
float accel_offset[3] = {0}, gyro_offset[3] = {0}, mag_offset[3] = {0};

float KalmanAngleRoll = 0, KalmanUncertaintyRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyPitch = 2 * 2;
float Kalman1DOutput[2] = {0, 0};

unsigned long last_time = 0;
float dt = 0.004;  // initial guess
double integral, previous, output, output_roll = 0;
double kp, ki, kd;
double setpoint = 0;


void setup() {
  Serial.begin(115200);
  kp = 1.5;
  ki = 0.1;
  kd = 0.01;

  if (!lsm.begin()) {
    Serial.println("Failed to detect LSM9DS1!");
    while (1);
  }

  // Sensor configuration
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  calibrateSensors(2000);
  last_time = micros();
}

void loop() {
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

  double actual = KalmanAnglePitch;
  double error = setpoint - actual;
  double actual_roll = KalmanAngleRoll;
  double error_roll = setpoint - actual_roll;
  output = pid(error);
  output_roll = pid(error_roll);
  if (output < 0){analogWrite(backRight,min(255.0,abs(output)));analogWrite(backLeft,min(255.0,abs(output)));}
  if (output >= 0){analogWrite(frontRight,min(255.0,output));analogWrite(frontLeft,min(255.0,output));}

  // if (output_roll < 0){analogWrite(backRight,min(255.0,abs(output_roll)));analogWrite(frontRight,min(255.0,abs(output_roll)));}
  // if (output_roll >= 0){analogWrite(backLeft,min(255.0,output_roll));analogWrite(frontLeft,min(255.0,output_roll));}


  Serial.print(KalmanAngleRoll);
  Serial.print("\t");
  Serial.println(KalmanAnglePitch);
  Serial.print("\t");
  Serial.println(output);
  Serial.print("\t");
  Serial.println(output_roll);

  delay(5); 
}

void calibrateSensors(int samples) {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0;

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

    mx += m.magnetic.x;
    my += m.magnetic.y;
    mz += m.magnetic.z;

    delay(1);
  }

  accel_offset[0] = ax / samples;
  accel_offset[1] = ay / samples;
  accel_offset[2] = az / samples - 9.81;  

  gyro_offset[0] = gx / samples;
  gyro_offset[1] = gy / samples;
  gyro_offset[2] = gz / samples;

  mag_offset[0] = mx / samples;
  mag_offset[1] = my / samples;
  mag_offset[2] = mz / samples;

  Serial.println("Calibration complete.");
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * 4 * 4;

  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}