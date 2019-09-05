#include "thermistor.h"

#include <Ticker.h>

#include <keylib.h>
const uint8_t oline[] = {11, 12, 13};
const uint8_t iline[] = {8, 9, 10};
readKey keylib = readKey();

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_ADDRESS 0x3c
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

bool heating = false;
bool cooling = false;

// peltier
const int COOLER_PIN = 2;
const int HEATER_PIN = 3;

// NTC
const int TEMP_PIN = A2;

// Fan
const int FAN_PIN = 4;

#define UNKNOWN_TEMP -10000

double temp = UNKNOWN_TEMP, temp2set = UNKNOWN_TEMP;
const int max_temp = 100, min_temp = 0;
double v, r;

#define TEMP_HISTORY_SIZE 128
int8_t temp_history[TEMP_HISTORY_SIZE];
int temp_history_count = 0;
int plot_max_temp = UNKNOWN_TEMP;
int plot_min_temp = UNKNOWN_TEMP;

Thermistor thermistor;

double Vcc = 3.3;
double Vref = 1.1;
const double T_0 = 273.15;
const double T_25 = T_0 + 25;
// 100k NTC
const double beta = 3950;
const double R_25 = 100000L; // 100k ohm
const unsigned long Rs = 470000L;

void display_all()
{
	display.clearDisplay();
	if(temp != UNKNOWN_TEMP) {
		display.setCursor(30, 0);
		display.print(F("= "));
		display.print(temp);
		display.print(F(" C"));
	}
	if(temp2set != UNKNOWN_TEMP) {
		display.setCursor(30, 12);
		display.print(F("> "));
		display.print(temp2set);
		display.print(F(" C"));
	}
	if(heating) {
		display.fillTriangle(2, 12, 12, 2, 22, 12, INVERSE);
	} else {
		display.fillTriangle(2, 2, 12, 12, 22, 2, INVERSE);
	}
	if(plot_min_temp != UNKNOWN_TEMP && plot_max_temp != UNKNOWN_TEMP) {
		Serial.print(F("history size:"));
		Serial.println(temp_history_count);
		for(int i=0; i<temp_history_count; i++) {
			display.writePixel(i, map(temp_history[i], min(20, plot_min_temp), min(60, plot_max_temp), 0, 64-24), INVERSE);
		}
	}
	display.display();
}

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

double read_temp()
{
//	Serial.print(F("TEMP_PIN value:"));
//	Serial.println(analogRead(TEMP_PIN));
	v = Vref*tempAnalogRead()/1024.;
	r = v/((Vcc-v)/Rs);
//	Serial.print(F("R:"));
//	Serial.println(r);
//	Serial.print(F("V:"));
//	Serial.println(v);
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
}

void update_temp()
{
	double tempC = thermistor.readTempC();
	Serial.print("tempC=");
	Serial.println(tempC);

	temp = read_temp();
	Serial.print(F("Temp:"));
	Serial.println(temp);
	if(temp2set == UNKNOWN_TEMP) {
		temp2set = temp;
	}
	if(plot_min_temp == UNKNOWN_TEMP || plot_min_temp > temp) {
		plot_min_temp = temp;
	}
	if(plot_max_temp == UNKNOWN_TEMP || plot_max_temp < temp) {
		plot_max_temp = temp;
	}
}

void heat_cool_proc()
{
	if(heating) {
		digitalWrite(HEATER_PIN, HIGH);
		digitalWrite(FAN_PIN, LOW);
		Serial.println(F("heating"));
	} else {
		digitalWrite(HEATER_PIN, LOW);
		if(cooling) {
			digitalWrite(COOLER_PIN, HIGH);
			digitalWrite(FAN_PIN, HIGH);
			Serial.println(F("cooling"));
		} else {
			digitalWrite(COOLER_PIN, LOW);
		}
	}
}

void update_history_proc()
{
	if(temp != UNKNOWN_TEMP) {
		if(temp_history_count >= TEMP_HISTORY_SIZE) {
			for(int i=1; i<TEMP_HISTORY_SIZE; i++) {
				temp_history[i-1] = temp_history[i];
			}
			temp_history_count--;
		}
		temp_history[temp_history_count++] = temp;
	}
}

String inputString;         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void parse_line(String line, String &command, String *args, int max_arg_count, int &args_count)
{
	command = "";
	args_count = 0;
	line.trim();
	if(line.length() > 0) {
		args_count = 0;
		int space_pos = 0;
		while(space_pos >= 0 && args_count < max_arg_count) {
			space_pos = line.indexOf(' ');
			if(command == "") {
				command = line.substring(0, space_pos);
				command.toUpperCase();
			} else {
				args[args_count++] = line.substring(0, space_pos);
			}
			if(space_pos >= 0) {
				line = line.substring(space_pos+1);
			}
		}
	}
}

#define MAX_ARGS_COUNT 2
String command;
String args[MAX_ARGS_COUNT];
int args_count;
String command_error;

void process_proc()
{
	if(stringComplete) {
		command_error = "";
		if(inputString.length() > 0) {
			// get command
			parse_line(inputString, command, args, MAX_ARGS_COUNT, args_count);
			if(command == "T") {
				if(args_count != 1) {
					command_error = F("T command has one parameter");
				} else {
					Serial.print(F("Setting temp:"));
					Serial.println(args[0]);
					temp2set = max(min_temp, args[0].toInt());
					Serial.print(F("temp2set:"));
					Serial.println(temp2set);
				}
			} else {
				command_error = "Unknown command line:" + inputString;
			}
		}
		if(command_error.length() > 0) {
			Serial.println("ERR " + command_error);
		} else {
			Serial.println(F("OK"));
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

Ticker heat_cool_timer(heat_cool_proc, 2000, 0, MILLIS);
Ticker process_timer(process_proc, 500, 0, MILLIS);
Ticker display_timer(display_all, 500, 0, MILLIS);
Ticker temp_history_timer(update_history_proc, 1000, 0, MILLIS);

void serialEvent()
{
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		if(stringComplete && isalnum(inChar)) {
			stringComplete = false;
			inputString = "";
Serial.println(F("Reset"));
		}
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag so the main loop can
		// do something about it:
		if (inChar == '\n') {
			stringComplete = true;
Serial.println(F("Complete"));
		}
	}
}

void process_keys()
{
	static byte last_key = 0;
	byte key = keylib.read_key_debounce();
	if(last_key != key) {
		if(temp2set != UNKNOWN_TEMP) {
			switch(key) {
				case 0x21:
					temp2set++;
					break;
				case 0x23:
					temp2set--;
					break;
			}
		}
		Serial.println(key, HEX);
		last_key = key;
	}
}

void setup()
{
	Serial.begin(115200);
	keylib.begin(sizeof(oline), sizeof(iline), oline, iline);
	pinMode(COOLER_PIN, OUTPUT);
	digitalWrite(COOLER_PIN, LOW);
	pinMode(HEATER_PIN, OUTPUT);
	digitalWrite(HEATER_PIN, LOW);
	pinMode(FAN_PIN, OUTPUT);
	digitalWrite(FAN_PIN, LOW);
	analogReference(INTERNAL);

	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { // Address 0x3D for 128x64
		Serial.println(F("SSD1306 allocation failed"));
	}
//	display.display();
//	delay(1000);

	// Show image buffer on the display hardware.
	// Since the buffer is intialized with an Adafruit splashscreen
	// internally, this will display the splashscreen.
	display.clearDisplay();
	display.display();
	// text display tests
	display.setTextSize(1);
	display.setTextColor(WHITE);
//	display.setCursor(30, 10);
//	display.print(F("I an a thermo"));
	display.display();

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
	thermistor.begin(TEMP_PIN, Vcc, Vref, 1023, Rs, R_25, 25, beta, 5, 40);


	heat_cool_timer.start();
	process_timer.start();
	temp_history_timer.start();
	display_timer.start();
	inputString.reserve(100);
}

void loop()
{
	process_keys();
	heat_cool_timer.update();
	process_timer.update();
	temp_history_timer.update();
	display_timer.update();
}
