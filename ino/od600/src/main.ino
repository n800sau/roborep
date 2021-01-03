/*
1602, HD44780 LCD
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 7
 * LCD D5 pin to digital pin 6
 * LCD D6 pin to digital pin 5
 * LCD D7 pin to digital pin 4
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

#include <Wire.h>
#include <LiquidCrystal.h>

#define   CONTRAST_PIN   9
#define   BACKLIGHT_PIN  10
#define   CONTRAST       80
#define   LIGHT_PIN      8

// DAC0 = D4

const int rs = 12, en = 11, d4 = 7, d5 = 6, d6 = 5, d7 = 3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7, BACKLIGHT_PIN, POSITIVE);

int min_transmission, max_transmission;

#define VALUE_PIN A0

void setup()
{
	analogReference(DEFAULT); // 5v
//	analogReference(INTERNAL);
//	analogReference(INTERNAL4V096);
	pinMode(VALUE_PIN, ANALOG);
	pinMode(LIGHT_PIN, OUTPUT);
	digitalWrite(LIGHT_PIN, LOW);
	Serial.begin(115200);
	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Switch on the backlight and LCD contrast levels
	pinMode(CONTRAST_PIN, OUTPUT);
	pinMode(CONTRAST_PIN, OUTPUT);
	analogWrite(CONTRAST_PIN, CONTRAST);
	lcd.backlight();
	// calibration
	digitalWrite(LIGHT_PIN, LOW);
	delay(100);
	min_transmission = analogRead(VALUE_PIN);
	digitalWrite(LIGHT_PIN, HIGH);
	delay(100);
	max_transmission = analogRead(VALUE_PIN);
	Serial.print("max val:");
	Serial.println(max_transmission);
	Serial.print("min val:");
	Serial.println(min_transmission);
}

void loop()
{
	digitalWrite(LIGHT_PIN, HIGH);
	lcd.setCursor(0, 0);
	lcd.print("*");
	delay(100);
	int v = analogRead(VALUE_PIN);
	lcd.clear();
	lcd.setCursor(6, 1);
	lcd.print("Raw:");
	lcd.print(v);
	// the optical density (O.D.) is a logarithmic measurement of the percent
	// transmission (%T) and it can be represented by the equation, A = log10 100 / %T
	float perc = 100 * float(v - min_transmission) / (max_transmission - min_transmission);
	float fv = log10((max_transmission - min_transmission) / float(v - min_transmission));
	lcd.setCursor(1, 1);
	lcd.print((int)(perc > 100 ? 100 : perc));
	lcd.print("%");
	lcd.setCursor(1, 0);
	lcd.print("OD600:");
	lcd.setCursor(10, 0);
	lcd.print(fv);
	digitalWrite(LIGHT_PIN, LOW);
	Serial.print("v = ");
	Serial.println(fv);
	delay(1000);
}
