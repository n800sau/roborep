#include <Ticker.h>
#include <Wire.h>
//TwoWire WireA(2);

#define WireA Wire

//#include <SoftWire.h>
//SoftWire WireA(PB10, PB11);

const byte I2C_SLAVE_ADDRESS = 8;
const byte I2C_REG_SET_TEMP = 0x01;
const byte I2C_REG_RESTART = 0x02;
const byte I2C_REG_READ_PWM = 0x10;
const byte I2C_REG_READ_TEMP = 0x11;

#include <keylib.h>
const uint8_t oline[] = {PA0, PA1, PA2};
const uint8_t iline[] = {PA3, PA4, PA5};
readKey keylib = readKey();

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_ADDRESS 0x3c
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &WireA);

#define MAX_PWM 400
#define MIN_PWM -400

// NTC
const int TEMP_PIN = PB1;

#define UNKNOWN_VAL -1000
const int min_temp = 0, max_temp = 100;

double temp = UNKNOWN_VAL, control_temp = UNKNOWN_VAL, temp2set = UNKNOWN_VAL;
int16_t heater_pwm = 0;
double v, r;

#define TEMP_HISTORY_SIZE 128
int8_t temp_history[TEMP_HISTORY_SIZE];
uint16_t temp_history_count = 0;
int32_t plot_max_temp = UNKNOWN_VAL;
int32_t plot_min_temp = UNKNOWN_VAL;

float Vcc = 3.3;
float Vref = Vcc;
const float T_0 = 273.15;
const float T_25 = T_0 + 25;
// 100k NTC
const float beta = 3950;
const float R_25 = 100000L; // 100k ohm
const unsigned long Rs = 470000L;

void init_slave()
{
	WireA.beginTransmission(I2C_SLAVE_ADDRESS);
	WireA.write(I2C_REG_RESTART);
	WireA.endTransmission();
}

int16_t read_slave(byte reg)
{
	int16_t rs = UNKNOWN_VAL;
	WireA.beginTransmission(I2C_SLAVE_ADDRESS);
	WireA.write(reg);
	WireA.endTransmission();

	delayMicroseconds(100);
	WireA.requestFrom(I2C_SLAVE_ADDRESS, 2);
	int available = WireA.available();
	if(2 == available)    // if two bytes were received
	{
		rs = WireA.read();  // receive high byte (overwrites previous reading)
		rs = rs << 8;    // shift high byte to be high 8 bits
		rs |= WireA.read(); // receive low byte as lower 8 bits
	} else {
		Serial.print("Available:");
		Serial.println(available);
	}
	return rs;
}

void display_status()
{
	display.clearDisplay();
	if(temp != UNKNOWN_VAL) {
		display.setCursor(30, 0);
		display.print(F("="));
		display.print(temp);
		display.print(F("C"));
		Serial.print(F("T "));
		Serial.println(temp);
	}
	if(temp2set != UNKNOWN_VAL) {
		display.setCursor(30, 12);
		display.print(F(">"));
		display.print(temp2set);
		display.print(F("C"));
		Serial.print(F("> "));
		Serial.println(temp2set);
	}

	Serial.print(F("PWM:"));
	Serial.println(heater_pwm);

	if(heater_pwm < 0) {
		display.setCursor(3, 14);
		display.print(map(abs(heater_pwm), 0, abs(MAX_PWM), 0, 100));
		display.fillTriangle(2, 12, 12, 2, 22, 12, WHITE);
	}
	if(heater_pwm > 0) {
		display.setCursor(3, 1);
		display.print(map(abs(heater_pwm), 0, abs(MIN_PWM), 0, 100));
		display.fillTriangle(2, 2+12, 12, 12+12, 22, 2+12, WHITE);
	}
	if(plot_min_temp != UNKNOWN_VAL && plot_max_temp != UNKNOWN_VAL) {
//		Serial.print(F("history size:"));
//		Serial.println(temp_history_count);
//		Serial.print(F("min temp:"));
//		Serial.println(plot_min_temp);
//		Serial.print(F("max temp:"));
//		Serial.println(plot_max_temp);
		for(int i=1; i<temp_history_count; i++) {
			display.drawLine(i-1, map(temp_history[i-1], plot_min_temp-1, plot_max_temp+1, 63, 24), i, map(temp_history[i], plot_min_temp-1, plot_max_temp+1, 63, 24), WHITE);
		}
	}
	display.display();
}

int tempAnalogRead(int temp_pin)
{
	int val = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(temp_pin);
		delay(1);
	}
	val = val / 20;
	return val;
}

double read_temp(int temp_pin)
{
	v = Vref*tempAnalogRead(temp_pin)/4096.;
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
	control_temp = read_temp(TEMP_PIN);
//	Serial.print(F("Control temp:"));
//	Serial.println(control_temp);

	int16_t val = read_slave(I2C_REG_READ_TEMP);
	if(val != UNKNOWN_VAL) {
		temp = val;
	}
	val = read_slave(I2C_REG_READ_PWM);
	if(val != UNKNOWN_VAL) {
		heater_pwm = val;
	}

	if(temp2set == UNKNOWN_VAL) {
		set_temp2set(min(max(temp, min_temp), max_temp));
		Serial.println(F("Ready"));
	}
	if(plot_min_temp == UNKNOWN_VAL || plot_min_temp > temp) {
		plot_min_temp = temp;
	}
	if(plot_max_temp == UNKNOWN_VAL || plot_max_temp < temp) {
		plot_max_temp = temp;
	}
	if(plot_min_temp > temp2set) {
		plot_min_temp = temp2set;
	}
	if(plot_max_temp < temp2set) {
		plot_max_temp = temp2set;
	}
}

void update_history_proc()
{
	if(temp != UNKNOWN_VAL) {
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
//			Serial.print("space_pos: ");
//			Serial.println(space_pos);
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

void set_temp2set(int t)
{
	temp2set = t;
	WireA.beginTransmission(I2C_SLAVE_ADDRESS);
	WireA.write(I2C_REG_SET_TEMP);
	WireA.write(byte(temp2set));
	WireA.endTransmission();      // stop transmitting
}

#define MAX_ARGS_COUNT 2
String command;
String args[MAX_ARGS_COUNT];
int args_count;
String command_error;

void parse_command_proc()
{
	if(stringComplete) {
		if(inputString.length() > 0) {
			command_error = "";
			// get command
			parse_line(inputString, command, args, MAX_ARGS_COUNT, args_count);
			Serial.print(F("command:"));
			Serial.println(command);
			Serial.print(F("args count:"));
			Serial.println(args_count);
			if(command == "T") {
				if(args_count != 1) {
					command_error = F("T command has one parameter");
				} else {
					Serial.print(F("Setting temp2set:"));
					Serial.println(args[0]);
					set_temp2set(max(min_temp, args[0].toInt()));
					Serial.print(F("temp2set:"));
					Serial.println(temp2set);
				}
			} else {
				command_error = "Unknown command line:" + inputString;
			}
			if(command_error.length() > 0) {
				Serial.println("ERR " + command_error);
			} else {
				Serial.print(F("OK "));
				Serial.println(inputString);
			}
			inputString = "";
			stringComplete = false;
		}
	}
}

void process_proc()
{
	update_temp();
}

Ticker status_timer(display_status, 1000, 0, MILLIS);
Ticker temp_history_timer(update_history_proc, 1000, 0, MILLIS);

void process_serial()
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
		if(temp2set != UNKNOWN_VAL) {
			switch(key) {
				case 0x23:
					temp2set++;
					break;
				case 0x13:
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

	Serial.println("Hello");
	WireA.begin();
	init_slave();

	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { // Address 0x3D for 128x64
		Serial.println(F("SSD1306 allocation failed"));
	}


	// Show image buffer on the display hardware.
	// Since the buffer is intialized with an Adafruit splashscreen
	// internally, this will display the splashscreen.
	display.clearDisplay();
	display.display();
	// text display tests
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.display();

	temp_history_timer.start();
	status_timer.start();
	inputString.reserve(30);
}

void loop()
{
	process_serial();
	parse_command_proc();
	process_keys();
	process_proc();
	temp_history_timer.update();
	status_timer.update();
}
