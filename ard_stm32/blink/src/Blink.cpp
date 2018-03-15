/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

#define LED_BUILTIN PC13

void setup()
{
	Serial.begin(115200);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

int i = 0;

void loop()
{
	Serial.println(i++);
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(1000);
}
