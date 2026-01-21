#include <Nicla_Vision_System.h>
#include <Portenta_System.h>
#include <STM32H747_System.h>

#include "Arduino.h"

// Define the hardware serial object
#define NICLA_HW_SERIAL Serial1


void setup() {
  // Initialize the USB serial port for debugging
  Serial.begin(19200);
  while (!Serial); // Wait for the serial port to connect

  // Initialize the hardware serial port for the loopback test
  NICLA_HW_SERIAL.begin(19200); // 115200 is a common baud rate for this test
  Serial.println("Hardware serial loopback test initiated.");
  Serial.println("Ensure TX1 and RX1 are connected with a jumper wire.");
}

void loop() {
  // Define the message to send
  String message = "Hello from Nicla!";
  
  // Send the message over hardware serial
  NICLA_HW_SERIAL.println(message);
  Serial.println("Message sent: " + message);

  // Wait for the message to be looped back
  unsigned long startTime = millis();
  String receivedMessage = "";
  while (millis() - startTime < 1000) { // Wait up to 1 second
    if (NICLA_HW_SERIAL.available()) {
      receivedMessage = NICLA_HW_SERIAL.readStringUntil('\n');
      break; // Exit the loop once a message is received
    }
  }
  
  // Check if the message was received successfully
  if (receivedMessage.length() > 0) {
    // Trim the received message to remove any extra whitespace
    receivedMessage.trim();
    if (receivedMessage.equals(message)) {
      Serial.println("Success! Message received: " + receivedMessage);
    } else {
      Serial.println("Mismatch! Received: " + receivedMessage);
    }
  } else {
    Serial.println("Timeout! No message received.");
  }
  
  delay(3000); // Wait 3 seconds before the next test
}
