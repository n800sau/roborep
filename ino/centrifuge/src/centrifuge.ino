#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// 83.5 mm diameter
// 1.118 * (D / 2) * (RPM/1000)^2 = G
// 1.118 * (D / 2) * (COUNTER*60/1000)^2 = G

#define DEBUG

#define LED_PIN 13

#define DIAM 83.5
#define COEF (1.118 * DIAM / 2  * 60 * 60 / 1000 / 1000)

// COEF * COUNTER**2 = G

const int QRE1113_PIN = A0; // Sensor output voltage

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() 
{
#ifdef DEBUG
	Serial.begin(115200);
#endif
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	pinMode(QRE1113_PIN, INPUT);
	lcd.begin();
	lcd.clear();
	lcd.print("RPM:");
	lcd.setCursor(0, 1);
	lcd.print("G:");
	lcd.backlight();
}

unsigned long ms = millis();
int counter = 0;
bool on = false;
bool led_pin_on = false;

void loop() 
{
	// Read in the ADC and convert it to a voltage:
	int v = analogRead(QRE1113_PIN);
//	Serial.println(v);
	if(v < 300) {
		if(on) {
			on = false;
			counter++;
		}
	} else if(v > 600) {
		if(!on) {
			on = true;
		}
	}
	unsigned long t = millis();
	if(t > ms + 1000) {
		lcd.setCursor(5, 0);
		lcd.print(counter * 60);
		lcd.print("   ");
		lcd.setCursor(5, 1);
		lcd.print(counter * counter * COEF);
		lcd.print("   ");
#ifdef DEBUG
		Serial.println(counter);
#endif
		led_pin_on = !led_pin_on;
		digitalWrite(LED_PIN, led_pin_on ? HIGH : LOW);
		ms = millis();
		counter = 0;
	}
	delayMicroseconds(1);
}

