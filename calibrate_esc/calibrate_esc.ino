#define ESC_PIN 18  // GPIO connected to ESC signal wire

// Convert microseconds to 16-bit duty cycle for 50Hz
uint32_t usToDuty(int microseconds) {
  return (microseconds * 65535L) / 20000;
}

void setup() {
  Serial.begin(115200);

  // Attach PWM on GPIO 18, 50Hz, 16-bit resolution
  ledcAttach(ESC_PIN, 50, 16);
  
  Serial.println("=== ESC CALIBRATION START ===");
  Serial.println("Step 1: Sending MAX throttle (2000 µs)");
  ledcWrite(ESC_PIN, usToDuty(2000));  // Max throttle
  delay(3000);  // Wait 3s for ESC to detect max signal

  Serial.println("Step 2: Sending MIN throttle (1000 µs)");
  ledcWrite(ESC_PIN, usToDuty(1000));  // Min throttle
  delay(3000);  // Wait for ESC to confirm

  Serial.println("=== ESC CALIBRATION COMPLETE ===");

  // Optional: idle at 1200 to gently spin motor
  ledcWrite(ESC_PIN, usToDuty(1200));
}

