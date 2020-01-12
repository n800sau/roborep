/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

//#define LED_PIN PC13
#define LED_PIN PA4

void setup()
{
	Serial.begin(115200);
  // initialize LED digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
}

int i = 0;

void loop()
{
	Serial.println(i++ * 100);
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_PIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_PIN, LOW);
   // wait for a second
  delay(1000);
}
