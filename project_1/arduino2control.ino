#include <SoftwareSerial.h>

SoftwareSerial hc12(10, 11); 

const int potPin = A0;
int lastPower = -1;
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200); 
  hc12.begin(115200);   
  Serial.println("Control Remoto RF Listo");
}

void loop() {
  while (hc12.available()) {
    Serial.write(hc12.read());
  }

  if (millis() - lastSend > 100) {
    int potValue = analogRead(potPin);
    int power = map(potValue, 0, 1023, 0, 100);
    
    if (abs(power - lastPower) >= 1) {
      hc12.print("S");
      hc12.println(power > 2 ? 1 : 0); 
      hc12.print("P");
      hc12.println(power);
      lastPower = power;
    }
    lastSend = millis();
  }
}