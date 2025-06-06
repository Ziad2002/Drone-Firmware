#define ESC_PIN 16
bool calibrate = true;

// Convert µs to 12-bit duty for 50Hz (12-bit max = 4095)
uint32_t usToDuty(int microseconds) {
  return (microseconds * 4095L) / 20000;
}

void setup() {
  Serial.begin(115200);

  // New API: Attach and configure in one call
  ledcAttach(ESC_PIN, 50, 12);  // pin, frequency, resolution

  if (calibrate) {
    Serial.println("=== ESC CALIBRATION START ===");

    Serial.println("Step 1: Sending MAX throttle (2000 µs)");
    ledcWrite(ESC_PIN, usToDuty(2000));
    delay(4000);

    Serial.println("Step 2: Sending MIN throttle (1000 µs)");
    ledcWrite(ESC_PIN, usToDuty(1400));
    delay(4000);

    Serial.println("=== ESC CALIBRATION COMPLETE ===");
  }

  ledcWrite(ESC_PIN, usToDuty(1000));  // Arm ESC
  delay(2000);
  Serial.println("ESC Armed");
}

void loop() {
  for (int us = 1000; us <= 2000; us++) {
    ledcWrite(ESC_PIN, usToDuty(us));
    delay(20);
    Serial.println(us);
  }

  delay(1000);

  for (int us = 2000; us >= 1000; us--) {
    ledcWrite(ESC_PIN, usToDuty(us));
    delay(20);
    Serial.println(us);
  }

  delay(1000);
}
