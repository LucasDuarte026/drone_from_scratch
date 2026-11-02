/**
 * ============================================================================
 * Generic I2C Bus Scanner & Line Diagnostic Tool
 *
 * Purpose:
 *   - Verifies physical pull-up states on SDA (A4) and SCL (A5)
 *   - Scans all 7-bit I2C addresses (0x01 to 0x7E) at 100 kHz
 *   - Reports responding devices (ACK) and bus transmission errors
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(57600);
  while (!Serial && millis() < 1500);

  Serial.println(F("\n=================================================="));
  Serial.println(F("          GENERIC I2C BUS DIAGNOSTIC FINDER       "));
  Serial.println(F("=================================================="));

  // 1. Check physical pull-up state on SDA (A4) and SCL (A5)
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(10);
  bool sda_high = digitalRead(A4);
  bool scl_high = digitalRead(A5);

  Serial.print(F("Pin Line Check: SDA (A4) = "));
  Serial.print(sda_high ? F("HIGH [OK]") : F("LOW [SHORT/STUCK!]"));
  Serial.print(F(" | SCL (A5) = "));
  Serial.println(scl_high ? F("HIGH [OK]") : F("LOW [SHORT/STUCK!]"));

  if (!sda_high || !scl_high) {
    Serial.println(F("[WARNING] Bus line stuck LOW. Check physical wiring or short circuit."));
  }

  // 2. Start I2C in standard 100 kHz mode
  Wire.begin();
  Wire.setClock(100000);
  Serial.println(F("I2C bus initialized at 100 kHz. Scanning...\n"));
}

void loop() {
  uint8_t count = 0;
  Serial.println(F("--- I2C Bus Scan (0x01 .. 0x7E) ---"));

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      count++;
      Serial.print(F("[FOUND] I2C device at 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    } else if (error == 4) {
      Serial.print(F("[ERROR] Bus fault at 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }
  }

  if (count == 0) {
    Serial.println(F("[NONE] No I2C devices responded on the bus."));
  } else {
    Serial.print(F("Total I2C devices found: "));
    Serial.println(count);
  }

  Serial.println(F("Next scan in 2.0s...\n"));
  delay(2000);
}
