#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#define LED 2

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

float accel_offset[3] = {0,0,0};
float gyro_offset[3] = {0,0,0};



void setupSensor()
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  
}


void setup() 
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  if (!lsm.begin())
  {
    Serial.println("Unable to initialize the LSM9DS1.");
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  Serial.println("LSM9DS1 Setup Completed.");

  calibrateIMU();
}

void loop() {
  lsm.read();

  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  float AccX = a.acceleration.x - accel_offset[0];
  float AccY = a.acceleration.y - accel_offset[1];
  float AccZ = a.acceleration.z - accel_offset[2];

  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch = atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

  Serial.print("Acceleration X [g]= ");
  Serial.print(AccX);
  Serial.print("Acceleration Y [g]= ");
  Serial.print(AccY);
  Serial.print("Acceleration Z [g]= ");
  Serial.print(AccZ);



  // float gx = g.gyro.x - gyro_offset[0];
  // float gy = g.gyro.y - gyro_offset[1];
  // float gz = g.gyro.z - gyro_offset[2];

  // Serial.print("Accel (m/s^2): ");
  // Serial.print(ax); Serial.print(", ");
  // Serial.print(ay); Serial.print(", ");
  // Serial.println(az);

  // Serial.print("Gyro (dps): ");
  // Serial.print(gx); Serial.print(", ");
  // Serial.print(gy); Serial.print(", ");
  // Serial.println(gz);

  delay(100);
}


void calibrateIMU() {
  const int samples = 5000;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println("Calibrating... Please keep the drone stationary.");
  digitalWrite(LED, HIGH);

  for (int i = 0; i < samples; i++){
    lsm.read();
    sensors_event_t a, m, g, temp;

    lsm.getEvent(&a, &m, &g, &temp);

    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;

    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;

    delay(5);
  }

  accel_offset[0] = ax_sum / samples;
  accel_offset[1] = ay_sum / samples;
  accel_offset[2] = az_sum / samples;

  gyro_offset[0] = gx_sum / samples;
  gyro_offset[1] = gy_sum / samples;
  gyro_offset[2] = gz_sum / samples;

  Serial.println("Calibration done.");
  digitalWrite(LED, 0);
}

