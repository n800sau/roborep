#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

#define PIN_COUNTER 2
#define PIN_BRIGHTNESS 10

// This Sketch counts the number of pulses a minute.
// Connect the GND on Arduino to the GND on the Geiger counter.
// Connect the 5V on Arduino to the 5V on the Geiger counter.
// Connect the VIN on the Geiger counter to the D2 on Arduino.

unsigned long counts; //variable for GM Tube events

unsigned long previousMillis; //variable for measuring time

void impulse()
{
	counts++;
}

#define LOG_PERIOD 60000 // count rate

void setup()
{
	counts = 0;
	Serial.begin(115200);
	pinMode(PIN_COUNTER, INPUT);
	pinMode(PIN_BRIGHTNESS, OUTPUT);
	analogWrite(PIN_BRIGHTNESS, 70);
	lcd.begin(16, 2);
	attachInterrupt(digitalPinToInterrupt(PIN_COUNTER), impulse, FALLING); //define external interrupts
	Serial.println("Start counter");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Counter");
}

void loop()
{
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis > LOG_PERIOD) {
		previousMillis = currentMillis;
		Serial.println(counts);
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(counts);
		lcd.setCursor(0, 1);
		lcd.print("count/min");
		counts = 0;
	}
}
