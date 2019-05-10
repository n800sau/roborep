#include "thermistor.h"



#include <Ticker.h>
#include <ACS712.h>

bool heating = false;
bool cooling = false;

const int COOLER_PIN = 2;
const int HEATER_PIN = 3;

const int CURRENT_SENSOR_PIN = A0;
const int TEMP_PIN = A1;

double temp = 0, temp2set = 0;
const int max_temp = 100, min_temp = 0;
double v, r;

ACS712 current_sensor(ACS712_05B, CURRENT_SENSOR_PIN);

Thermistor *thermistor;


double Vcc = 5;
double Vref = 1.1;
const double T_0 = 273.15;
const double T_25 = T_0 + 25;
// 100k
//const double beta = 3950;
//const double R_25 = 100000; // 100k ohm
//const unsigned int Rs = 270000;


// 10k
const double beta = 3435;
const double R_25 = 10000; // 10k ohm
//const unsigned int Rs = 32600;
const unsigned int Rs = 47000;

int tempAnalogRead()
{
	int val = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(TEMP_PIN);
		delay(1);
	}
	val = val / 20;
	return val;
}

double thermister(double r)
{
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
}

void update_temp()
{
//	Serial.print(F("TEMP_PIN value:"));
//	Serial.println(analogRead(TEMP_PIN));
	v = Vref*tempAnalogRead()/1024.;
	r = v/((Vcc-v)/Rs);
//	Serial.print(F("R:"));
//	Serial.println(r);
	temp = thermister(r);
	Serial.print(F("Temp:"));
	Serial.println(temp);
//	Serial.print(F("V:"));
//	Serial.println(v);

  double tempC = thermistor->readTempC();
  Serial.println("tempC=" + String(tempC));
}


void current_proc()
{
	if(digitalRead(HEATER_PIN) == HIGH) {
		Serial.print(F("Heating current:"));
	} else if(digitalRead(COOLER_PIN) == HIGH) {
		Serial.print(F("Cooling current:"));
	} else {
		Serial.print(F("Idle current:"));
	}
	Serial.println(current_sensor.getCurrentDC());
}

void heat_cool_proc()
{
	if(heating) {
		digitalWrite(HEATER_PIN, HIGH);
		Serial.println("heating");
	} else {
		digitalWrite(HEATER_PIN, LOW);
		if(cooling) {
			digitalWrite(COOLER_PIN, HIGH);
			Serial.println("cooling");
		} else {
			digitalWrite(COOLER_PIN, LOW);
		}
	}
}

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void process_proc()
{
	if(stringComplete) {
		if(inputString.length() > 0) {
			int t = inputString.toInt();
			if(t) {
				temp2set = max(min_temp, t);
				Serial.print(F("temp2set:"));
				Serial.println(temp2set);
			}
		}
		inputString = "";
		stringComplete = false;
	}
	update_temp();
	heating = false;
	cooling = false;
	if(temp < temp2set) {
		heating = true;
		cooling = false;
	} else if(temp > temp2set) {
		heating = false;
		cooling = true;
	}
}

Ticker heat_cool_timer(heat_cool_proc, 1000, 0, MILLIS);
Ticker current_timer(current_proc, 2000, 0, MILLIS);
Ticker process_timer(process_proc, 500, 0, MILLIS);

void serialEvent()
{
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		if(stringComplete && isalnum(inChar)) {
			stringComplete = false;
			inputString = "";
Serial.println("Reset");
		}
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag so the main loop can
		// do something about it:
		if (inChar == '\n') {
			stringComplete = true;
Serial.println("Complete");
		}
	}
}

void setup()
{
	Serial.begin(115200);
	pinMode(COOLER_PIN, OUTPUT);
	digitalWrite(COOLER_PIN, LOW);
	pinMode(HEATER_PIN, OUTPUT);
	digitalWrite(HEATER_PIN, LOW);
	analogReference(INTERNAL);

  /*
  * arg 1: pin: Analog pin
  * arg 2: vcc: Input voltage
  * arg 3: analogReference: reference voltage. Typically the same as vcc, but not always (ie ESP8266=1.0)
  * arg 4: adcMax: The maximum analog-to-digital convert value returned by analogRead (1023 or 4095)
  * arg 5: seriesResistor: The ohms value of the fixed resistor (based on your hardware setup, usually 10k)
  * arg 6: thermistorNominal: Resistance at nominal temperature (will be documented with the thermistor, usually 10k)
  * arg 7: temperatureNominal: Temperature for nominal resistance in celcius (will be documented with the thermistor, assume 25 if not stated)
  * arg 8: bCoef: Beta coefficient (or constant) of the thermistor (will be documented with the thermistor, typically 3380, 3435, or 3950)
  * arg 9: samples: Number of analog samples to average (for smoothing)
  * arg 10: sampleDelay: Milliseconds between samples (for smoothing)
  */

  // For 5V Arduino
  thermistor = new Thermistor(TEMP_PIN, Vcc, Vref, 1023, Rs, R_25, 25, beta, 5, 40);



	current_sensor.setZeroPoint(574);

//	Serial.println(F("Calibrating... Ensure that no current flows through the sensor at this moment"));
//	int zero = current_sensor.calibrate();
//	Serial.println(F("Done!"));
//	Serial.print(F("Zero point for this sensor = "));
//	Serial.println(zero);
	for(int i=0; i<2; i++) {
		Serial.print(F("No current:"));
		Serial.println(current_sensor.getCurrentDC());
		delay(1000);
	}
	heat_cool_timer.start();
	current_timer.start();
	process_timer.start();
	inputString.reserve(100);
}

void loop()
{
	heat_cool_timer.update();
	current_timer.update();
	process_timer.update();
}
