#include <SoftwareSerial.h>

SoftwareSerial niclaSerial(2, 3); //(RX, TX) so Nicla TX1 -> RX 'D2' and Nicla RX1 -> TX 'D3'

void setup() {
  Serial.begin(19200);
  niclaSerial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (niclaSerial.available()) {
    String msg = niclaSerial.readStringUntil('\n');

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    // SEND BACK TO NICLA
    niclaSerial.println(msg);

    Serial.print("Echoed: ");
    Serial.println(msg);
  }
}
