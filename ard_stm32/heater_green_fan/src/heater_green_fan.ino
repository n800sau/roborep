//#include "thermistor.h"
#include <VNH3SP30.h>
#include <RTClock.h>

#define USE_PID

#ifdef USE_PID

#include <PID_v1.h>

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double hKp=100, hKi=50, hKd=50;
double cKp=200, cKi=50, cKd=50;
PID heaterPID(&Input, &Output, &Setpoint, hKp, hKi, hKd, REVERSE);

#endif //USE_PID

//#include <Thermistor.h>
//#include <NTC_Thermistor.h>

#define SENSOR_PIN             PB1
#define REFERENCE_RESISTANCE   330000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950
#define STM32_ANALOG_RESOLUTION 4095

//NTC_Thermistor thermistor(
//    SENSOR_PIN,
//    REFERENCE_RESISTANCE,
//    NOMINAL_RESISTANCE,
//    NOMINAL_TEMPERATURE,
//    B_VALUE,
//    STM32_ANALOG_RESOLUTION // <- for a thermistor calibration
//);

#include <Ticker.h>

#include <keylib.h>
const uint8_t oline[] = {PA15, PB3, PB4};
const uint8_t iline[] = {PB5, PB6, PB7};
readKey keylib = readKey();

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

// ST7789 TFT module connections
#define TFT_CS     PA4
#define TFT_RST    PA2
#define TFT_DC     PA3
#define TFT_SCLK   PA5
#define TFT_MOSI   PA7

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int heater_pwm = 0;
const int MIN_PWM = -400, MAX_PWM = 400;

// peltier
VNH3SP30 Motor1;    // define control object for 1 motor

// motor pins
#define M1_PWM PA8    // pwm pin motor
#define M1_INA PA12    // control pin INA
#define M1_INB PA11    // control pin INB
#define M1_DIAG PB14   // diagnose pins (combined DIAGA/ENA and DIAGB/ENB)
#define M1_CS PB13    // current sense pin

#define PLOT_TOP 24
#define PLOT_BOTTOM 239
#define PLOT_LEFT 24
#define PLOT_HEIGHT (PLOT_BOTTOM - PLOT_TOP)
#define PIXELS_PER_STEP 12

GFXcanvas1 dbuf(240, PLOT_TOP);

// NTC
const int TEMP_PIN = PB1;
const int TEMP_CONTROL_PIN = PA6;
// 2.5v ref
const int REF_PIN = PB0;

// Fan
const int FAN_PIN = PB12;

// unknown temp marker
const int UNKNOWN_TEMP = -1000;
const int INITIAL_TEMP2SET = 25;

double temp = UNKNOWN_TEMP, temp2set = INITIAL_TEMP2SET, temp_control = UNKNOWN_TEMP;
const int32_t max_temp = 100, min_temp = 10;

#define TEMP_HISTORY_SIZE (240-PLOT_LEFT)
int8_t temp_control_history[TEMP_HISTORY_SIZE];
int8_t temp_history[TEMP_HISTORY_SIZE];
int8_t temp_req_history[TEMP_HISTORY_SIZE];
uint16_t temp_history_count = 0;
int32_t plot_max_temp = max_temp;
int32_t plot_min_temp = min_temp;

float Vcc = 3.3;
// ref from LM336-2.5
float Vref = 2.45;
const float T_0 = 273.15;
const float T_25 = T_0 + 25;
// 100k NTC
const float beta = 3950;
const float R_25 = 100000L; // 100k ohm
//const unsigned long Rs = 10000L;
const unsigned long Rs = 150000L;
//const unsigned long Rs = 330000L;
//const unsigned long Rs = 470000L;
// ************************************************

const int MAX_ITEM_PER_CYCLE = 5;
const int MAX_CYCLES = 10;

RTClock rtclock (RTCSEL_LSE); // initialise

enum DISPLAY_MODE_T {DISPLAY_NONE, DISPLAY_STATUS, DISPLAY_SETUP};

DISPLAY_MODE_T display_mode = DISPLAY_STATUS;
DISPLAY_MODE_T display_mode_env = DISPLAY_NONE;

int setup_edit_pos = 0;

typedef struct S_CYCLE_ITEM {
	time_t secs;
	byte temp;
	S_CYCLE_ITEM() {
		secs = 30;
		temp = 25;
	}
} t_cycle_item;

typedef struct S_CYCLE {
	t_cycle_item items[MAX_ITEM_PER_CYCLE];
	byte item_count;
	unsigned repeats;
	// dynamics
	byte current_item;
	unsigned current_repeat;
	S_CYCLE() {
		current_repeat = 0;
		current_item = 0;
		item_count = 1;
		repeats = 1;
		for(int i=0; i<item_count; i++) {
			items[i] = t_cycle_item();
		}
	}
} t_cycle;

typedef struct S_PROGRAM {
	t_cycle cycles[MAX_CYCLES];
	byte cycle_count;
	// dynamics
	bool running, finished;
	byte current_cycle;
	time_t last_switch_time;
	time_t time_left;
	S_PROGRAM()
	{
		last_switch_time = 0;
		time_left = 0;
		cycle_count = 3;
		for(int i=0; i<cycle_count; i++) {
			cycles[i] = t_cycle();
		}
		cycles[0].items[0].temp = 92;
		cycles[1].item_count = 3;
		cycles[1].items[0].temp = 92;
		cycles[1].items[1].temp = 72;
		cycles[1].items[2].temp = 54;
		cycles[1].repeats = 10;
		cycles[2].items[0].temp = 25;
		current_cycle = 0;
		running = false;
		finished = false;
	}
} t_program;

t_program program;

// ************************************************

int setup_edit_pos_count()
{
	int rs = 1;
	for(int i=0; i<program.cycle_count; i++) {
		rs += 1;
		rs += program.cycles[i].item_count * 2;
	}
	return rs;
}

void draw_plot_scale()
{
	int ptemp = plot_min_temp;
	//PLOT_HEIGHT
	int pstep = 20;
	while(ptemp < plot_max_temp) {
		ptemp += pstep;
		if(ptemp != plot_min_temp) {
			int y = map(ptemp, plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP);
			Serial.print(ptemp);
			Serial.print(" at ");
			Serial.println(y);
			display.setCursor(0, y);
			display.print(ptemp);
		}
	}
}

void display_status()
{
	if(display_mode_env != display_mode) {
		display.fillScreen(ST77XX_BLACK);
		display.setTextSize(1);
		display.setTextColor(ST77XX_WHITE);
		draw_plot_scale();
		display_mode_env = display_mode;
	}
	dbuf.fillScreen(0);
	if(temp != UNKNOWN_TEMP) {
		dbuf.setCursor(30, 0);
		dbuf.print(F("="));
		dbuf.print(temp);
		dbuf.print(F("C"));
		Serial.print(F("T "));
		Serial.println(temp);
	}
	if(temp_control != UNKNOWN_TEMP) {
		dbuf.setCursor(80, 0);
		dbuf.print(F("="));
		dbuf.print(temp_control);
		dbuf.print(F("C"));
		Serial.print(F("Control temp:"));
		Serial.println(temp_control);
	}
	dbuf.setCursor(30, 12);
	dbuf.print(F(">"));
	dbuf.print(temp2set);
	dbuf.print(F("C"));
	Serial.print(F("> "));
	Serial.println(temp2set);
	Serial.print(F("PWM:"));
	Serial.println(heater_pwm);
	if(heater_pwm < 0) {
		dbuf.setCursor(3, 14);
		dbuf.print(map(abs(heater_pwm), 0, abs(MAX_PWM), 0, 100));
		dbuf.fillTriangle(2, 12, 12, 2, 22, 12, 1);
	}
	if(heater_pwm > 0) {
		dbuf.setCursor(3, 1);
		dbuf.print(map(abs(heater_pwm), 0, abs(MIN_PWM), 0, 100));
		dbuf.fillTriangle(2, 2+12, 12, 12+12, 22, 2+12, 1);
	}
	dbuf.setCursor(130, 0);
	dbuf.print(program.running ? "Running" : (program.finished ? "Finished" : "Stopped"));
	if(program.running) {
		dbuf.setCursor(130, 12);
		dbuf.print(program.current_cycle);
		dbuf.print("-");
		dbuf.print(program.cycles[program.current_cycle].current_repeat);
		dbuf.print("-");
		dbuf.print(program.cycles[program.current_cycle].current_item);
		dbuf.print("-");
		dbuf.print(program.time_left);
//		Serial.print("Time left:");
//		Serial.println(program.time_left);
	}
	// copy buffer to screen
	display.drawBitmap(0, 0, dbuf.getBuffer(), dbuf.width(), dbuf.height(), ST77XX_YELLOW, ST77XX_BLACK);
//	Serial.print(F("history size:"));
//	Serial.println(temp_history_count);
//	Serial.print(F("min temp:"));
//	Serial.println(plot_min_temp);
//	Serial.print(F("max temp:"));
//	Serial.println(plot_max_temp);
	display.drawLine(PLOT_LEFT, PLOT_TOP, PLOT_LEFT, PLOT_BOTTOM, ST77XX_BLACK);
	for(int i=1; i<temp_history_count; i++) {
		// fill line black
		display.drawLine(PLOT_LEFT+i, PLOT_TOP, PLOT_LEFT+i, PLOT_BOTTOM, ST77XX_BLACK);
		display.drawLine(PLOT_LEFT+i-1, map(temp_control_history[i-1], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), PLOT_LEFT+i,
				map(temp_control_history[i], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), ST77XX_YELLOW);
		display.drawLine(PLOT_LEFT+i-1, map(temp_history[i-1], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), PLOT_LEFT+i,
				map(temp_history[i], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), ST77XX_BLUE);
		display.drawLine(PLOT_LEFT+i-1, map(temp_req_history[i-1], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), PLOT_LEFT+i,
				map(temp_req_history[i], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), ST77XX_GREEN);
	}
}

void display_setup() {
	if(display_mode_env != display_mode) {
		display.fillScreen(ST77XX_BLACK);
		dbuf.setTextSize(2);
		dbuf.setTextColor(1);
		display.setTextSize(2);
		display.setTextColor(ST77XX_GREEN);
		display.setCursor(0, 0);
		display.print(F("Program:"));
		display_mode_env = display_mode;
	}
	int y = 12;
	for(int i=0; i<program.cycle_count; i++) {
		t_cycle *c = &program.cycles[i];
		display.setCursor(10, y);
		dbuf.fillScreen(0);
		dbuf.setCursor(10, 0);
		dbuf.print(F("Repeats:"));
		dbuf.print(c->repeats);
		display.drawBitmap(0, y, dbuf.getBuffer(), dbuf.width(), 12, ST77XX_YELLOW, ST77XX_BLACK);
		y += 12;
		for(int j=0; j<c->item_count; j++) {
			t_cycle_item *ci = &c->items[j];
			dbuf.fillScreen(0);
			dbuf.setCursor(20, 0);
			dbuf.print(F("secs:"));
			dbuf.print(ci->secs);
			dbuf.print(F(", temp:"));
			dbuf.print(ci->temp);
			display.drawBitmap(0, y, dbuf.getBuffer(), dbuf.width(), 12, ST77XX_GREEN, ST77XX_BLACK);
			y += 12;
		}
	}
}

void update_display()
{
	switch(display_mode) {
		case DISPLAY_SETUP:
			display_setup();
			break;
		default:
			display_status();
			break;
	}
}

// NTC scheme
// GND - NTC - measure - Rs - Vref
double ntcVoltageRead(int temp_pin)
{
	double val = 0;
	double ref = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(temp_pin);
		ref += analogRead(REF_PIN);
		delay(1);
	}
//	Serial.print(F("val:"));
//	Serial.print(val);
//	Serial.print(F(", ref:"));
//	Serial.print(ref);
//	Serial.print(F(", ref V:"));
//	Serial.print(ref/4096/20*Vcc);
//	Serial.print(F(", NCC V:"));
//	Serial.println(val/4096/20*Vcc);
	// Voltage on NTC
	return Vref * val / ref;
}

double read_temp(int temp_pin)
{
	double v = ntcVoltageRead(temp_pin);
	// voltage on Rs
	double rV = Vref-v;
	// current
	double current = rV / Rs;
	// resistance of NTC
	double r = v / current;
	// temp on NTC
	double rs = 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
//	Serial.print(F("R:"));
//	Serial.println(r);
//	Serial.print(F("V:"));
//	Serial.println(v);
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return rs;
}

void update_temp()
{
	if(temp == UNKNOWN_TEMP) {
		Serial.println("Ready");
	}
	temp = read_temp(TEMP_PIN);
	//temp = thermistor.readCelsius();

	temp_control = read_temp(TEMP_CONTROL_PIN);
//	Serial.print(F("Temp:"));
//	Serial.println(temp);
}

void update_history_proc()
{
	if(temp != UNKNOWN_TEMP) {
		if(temp_history_count >= TEMP_HISTORY_SIZE) {
			for(int i=1; i<TEMP_HISTORY_SIZE; i++) {
				temp_history[i-1] = temp_history[i];
				temp_control_history[i-1] = temp_control_history[i];
				temp_req_history[i-1] = temp_req_history[i];
			}
			temp_history_count--;
		}
		temp_req_history[temp_history_count] = temp2set;
		temp_history[temp_history_count] = temp;
		temp_control_history[temp_history_count] = temp_control;
		temp_history_count++;
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
			if(space_pos<0) {
				space_pos = line.length();
			}
			//Serial.print("space_pos: ");
			//Serial.println(space_pos);
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

void set_temp2set(int temp)
{
	if(temp2set < temp) {
		// K for cooling
		heaterPID.SetTunings(cKp, cKi, cKd);
	} else {
		// K for heating
		heaterPID.SetTunings(hKp, hKi, hKd);
	}
	temp2set = temp;
}

#define MAX_ARGS_COUNT 5
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
			} else if(command == "Z") {
				// set program params
				// usage: Z <cycle count>
				if(args_count != 1) {
					command_error = F("usage: Z <cycle count>");
				} else {
					program.cycle_count = min(MAX_CYCLES, max(1, args[0].toInt()));
				}
			} else if(command == "X") {
				// set cycle params
				// usage: X <cycle index> <item_count> <repeats>
				if(args_count != 3) {
					command_error = F("usage: X <cycle index> <item_count> <repeats>");
				} else {
					int cycle_index = args[0].toInt();
					if(cycle_index < 0 || cycle_index >= program.cycle_count) {
						command_error = F("Cycle index out of range");
					} else {
						int item_count = args[1].toInt();
						if(item_count < 1 || item_count >= MAX_ITEM_PER_CYCLE) {
							command_error = F("item_count out of range");
						} else {
							int repeats = args[2].toInt();
							if(repeats < 1) {
								command_error = F("repeats out of range");
							} else {
								program.cycles[cycle_index].item_count = item_count;
								program.cycles[cycle_index].repeats = repeats;
							}
						}
					}
				}
			} else if(command == "V") {
				// set cycle item params
				// usage: V <cycle> <item> <temp> <secs>
				if(args_count != 4) {
					command_error = F("usage: V <cycle index> <item index> <temp> <secs>");
				} else {
					int cycle_index = args[0].toInt();
					if(cycle_index < 0 || cycle_index >= program.cycle_count) {
						command_error = F("Cycle index out of range");
					} else {
						int item_index = args[1].toInt();
						if(item_index < 0 || item_index >= program.cycles[cycle_index].item_count) {
							command_error = F("item_index out of range");
						} else {
							int temp = args[2].toInt();
							if(temp < min_temp || temp > max_temp) {
								command_error = F("temp out of range");
							} else {
								int secs = args[3].toInt();
								if(secs < 5 || secs > 3600) {
									command_error = F("secs out of range (5..3600)");
								} else {
									program.cycles[cycle_index].items[item_index].temp = temp;
									program.cycles[cycle_index].items[item_index].secs = secs;
								}
							}
						}
					}
				}
			} else if(command == "R") {
				// reset program
				program = t_program();
			} else if(command == "G") {
				// start program
				program.running = true;
			} else if(command == "P") {
				// pause program
				program.running = false;
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
#ifdef USE_PID
	Input = temp;
	Setpoint = temp2set;
//	Serial.print("Error:");
//	Serial.println(temp2set-temp);
	heaterPID.Compute();
	heater_pwm = Output;
#else
	if(temp < temp2set) {
		heater_pwm = MAX_PWM;
	} else if(temp > temp2set) {
		heater_pwm = MIN_PWM;
	}
#endif // USE_PID
	Motor1.setSpeed(heater_pwm);
//	if(heater_pwm > MAX_PWM/2) {
//		digitalWrite(FAN_PIN, LOW);
//	} else if(heater_pwm < 0) {
//		digitalWrite(FAN_PIN, HIGH);
//	}
}

void program_proc()
{
	if(program.running) {
		if(program.last_switch_time == 0) {
			program.last_switch_time = rtclock.now();
		}
		t_cycle *cy = &(program.cycles[program.current_cycle]);
		t_cycle_item *it = &(cy->items[cy->current_item]);
		time_t m = rtclock.now();
		program.time_left = max(0, program.last_switch_time + it->secs - m);
		if(program.time_left == 0) {
			program.last_switch_time = m;
			cy->current_item++;
			if(cy->current_item >= cy->item_count) {
				cy->current_item = 0;
				cy->current_repeat++;
				if(cy->current_repeat >= cy->repeats) {
					cy->current_repeat = 0;
					program.current_cycle++;
					if(program.current_cycle >= program.cycle_count) {
						program.current_cycle = 0;
						program.running = false;
						program.finished = true;
						temp2set = INITIAL_TEMP2SET;
					}
				}
			}
		}
		if(program.running) {
			// refine current item again
			cy = &(program.cycles[program.current_cycle]);
			it = &(cy->items[cy->current_item]);
			// and set desired temp
			temp2set = cy->items[cy->current_item].temp;
		}
	}
}

Ticker status_timer(update_display, 1000, 0, MILLIS);
Ticker temp_history_timer(update_history_proc, 1000, 0, MILLIS);
Ticker program_timer(program_proc, 1000, 0, MILLIS);

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
		switch(key) {
			case 0x23:
				temp2set++;
				break;
			case 0x13:
				temp2set--;
				break;
			case 0x11:
				display_mode = DISPLAY_SETUP;
				break;
			case 0x12:
				display_mode = DISPLAY_STATUS;
				break;
			case 0x21: // up
				setup_edit_pos++;
				if(setup_edit_pos >= setup_edit_pos_count()) {
					setup_edit_pos = setup_edit_pos_count();
				}
				break;
			case 0x22: // down
				setup_edit_pos--;
				if(setup_edit_pos < 0) {
					setup_edit_pos = 0;
				}
				break;
		}
		Serial.println(key, HEX);
		last_key = key;
	}
}

void setup()
{
	Serial.begin(115200);
	keylib.begin(sizeof(oline), sizeof(iline), oline, iline);
	pinMode(TEMP_PIN, INPUT);
	pinMode(TEMP_CONTROL_PIN, INPUT);
	pinMode(REF_PIN, INPUT);
	pinMode(FAN_PIN, OUTPUT);
	digitalWrite(FAN_PIN, LOW);
	digitalWrite(FAN_PIN, HIGH);

	Motor1.begin(M1_PWM, M1_INA, M1_INB, M1_DIAG, M1_CS);    // Motor 1 object connected through specified pins 

	display.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
	display.setRotation(2);

	temp_history_timer.start();
	status_timer.start();
	program_timer.start();
	inputString.reserve(30);

#ifdef USE_PID

	//tell the PID to range between 0 and the full window size
	heaterPID.SetOutputLimits(MIN_PWM, MAX_PWM);

	//turn the PID on
	heaterPID.SetMode(AUTOMATIC);
#endif // USE_PID
}

void loop()
{
	process_serial();
	parse_command_proc();
	process_keys();
	process_proc();
	temp_history_timer.update();
	program_timer.update();
	status_timer.update();
}
