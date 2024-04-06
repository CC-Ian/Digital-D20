#include <Arduino.h>

#define LED_ENABLE PA4

void setup() {
  pinMode(LED_ENABLE, OUTPUT);
  digitalWrite(LED_ENABLE, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  digitalWrite(LED_ENABLE, LOW);
  delay(500);
  digitalWrite(LED_ENABLE, HIGH);
}