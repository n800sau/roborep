/*
1602, HD44780 LCD
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 6

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


double Vcc = 3.39;
double Vref = 1.1;
const double T_0 = 273.15;
const double T_25 = T_0 + 25;

struct ntc_t {
	double beta;
	double R_25;
	long Rs;
	int pin;
	const char *name;
	// calculated value
	double temp;
};

// 100k
//const double beta = 3950;
//const double R_25 = 100000; // 100k ohm
//const unsigned int Rs = 270000;

ntc_t ntc3950_100 = {
	3950,
	100000,
	470000,
	A0,
	"3950_100"
};

// 10k
//const double beta = 3435;
//const double R_25 = 10000; // 10k ohm
//const unsigned int Rs = 47000;

ntc_t ntc3435_10 = {
	3435,
	10000,
	47000,
	A1,
	"3435_10"
};

int tempAnalogRead(int pin)
{
	int val = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(pin);
		delay(1);
	}
	val = val / 20;
//	return 1024;
//	return 10;
	return val;
}

double thermister(double r, ntc_t &t)
{
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return 1 / ((log(r / t.R_25) / t.beta) + 1/T_25) - T_0;
}

void update_temp(ntc_t &t)
{
	Serial.println(t.name);
	Serial.print(F("TEMP_PIN value:"));
	Serial.println(analogRead(t.pin));
	double v = Vref*tempAnalogRead(t.pin)/1024.;
	double r = v/((Vcc-v)/t.Rs);
	Serial.print(F("R:"));
	Serial.println(r);
	Serial.print(F("V:"));
	Serial.println(v);
	t.temp = thermister(r, t);
	Serial.print(F("Temp:"));
	Serial.println(t.temp);
	Serial.println();
}


void setup()
{
	Serial.begin(9600);
	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Print a message to the LCD.
	lcd.setCursor(0, 0);
	lcd.print("A:");
	lcd.setCursor(0, 1);
	lcd.print("B:");
	lcd.setCursor(9, 0);
	lcd.print("C:");
	lcd.setCursor(9, 1);
	lcd.print("D:");
	analogReference(INTERNAL);
	sensors.begin();
}

void loop()
{
	lcd.setCursor(2, 0);
	lcd.print(ntc3950_100.temp);
	lcd.setCursor(2, 1);
	lcd.print(ntc3435_10.temp);
	update_temp(ntc3950_100);
	update_temp(ntc3435_10);
	sensors.requestTemperatures(); // Send the command to get temperatures
	float tempC = sensors.getTempCByIndex(0);
	// Check if reading was successful
	if(tempC != DEVICE_DISCONNECTED_C) 
	{
		Serial.print("Temperature for the device 1 (index 0) is: ");
		Serial.println(tempC);
		lcd.setCursor(11, 0);
		lcd.print(tempC);
	} 
	else
	{
		lcd.setCursor(11, 0);
		lcd.print("???");
	}
	delay(1000);
}
