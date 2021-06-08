#include <ESP8266WiFi.h>
#include <FS.h>
#include <WiFiClient.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
//#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
//#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <FSWebServerLib.h>
#include <Hash.h>
#include <LittleFS.h>

#include <TaskScheduler.h>

#include <Wire.h>

#include <LiquidCrystal_PCF8574.h>
#define BACKLIGHT_ON false
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <SI7021.h>
SI7021 si;

#include "BMP280.h"
BMP280 bmp;

#define VERSION "0.6b"

const int HEATER_PIN = D8;
const int FAN_PIN = D7;

const int ROTARY_KEY = D0;
const int ROTARY_S1 = D5;
const int ROTARY_S2 = D6;

const int CLICKS_PER_STEP = 4;   // this number depends on your rotary encoder 

#include "Button2.h"
#include "ESPRotary.h"
ESPRotary rotary = ESPRotary(ROTARY_S1, ROTARY_S2, CLICKS_PER_STEP);
Button2 button = Button2(ROTARY_KEY);

// 80 - 8v
int max_heat_pwm = 100;
int max_fan_pwm = 70;

const char *pwm_command_prefix = "PWM";

int heater = max_heat_pwm;
int default_temp = 32;
unsigned long initial_millis = 0;

// degreese
int target = 0;
#define MAX_TEMP 50
#define MIN_TEMP 10

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=3000, Ki=0, Kd=0;

#include <PID_v1.h>
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;
float secondary_temp = 0;

#define HISTORY_SIZE 100
int history_count = 0;
uint8_t temp_history[HISTORY_SIZE];
uint8_t heater_history[HISTORY_SIZE];
uint8_t target_history[HISTORY_SIZE];
uint8_t secondary_temp_history[HISTORY_SIZE];


//#include <RemoteDebug.h>
//RemoteDebug Debug;

enum DISPLAY_MODE {
	DM_STATE,
	DM_MENU,
	DM_NETWORK,
	DM_WIFI
};

DISPLAY_MODE dmode = DM_STATE;

enum SMenuItemTypes {
	MI_PROCESS,
	MI_NETWORK,
	MI_WIFI,
	MI_COUNT
};

void on_show_process();
void on_show_ip();
void on_show_wifi();

struct sMenuItem {
	uint8_t id;
	String title;
	void (*select_fn)();
} menu_items[MI_COUNT] = {
	{ MI_PROCESS, "Show Process", on_show_process },
	{ MI_NETWORK, "Show Network", on_show_ip },
	{ MI_WIFI, "Show WiFi", on_show_wifi }
};

const uint8_t MENU_LINE_COUNT = 2;
uint8_t menu_top = 0;
uint8_t menu_pos = 0;

void menu_render()
{
	lcd.clear();
	for(int i=0; i<min(2, MI_COUNT-menu_top); i++) {
		if(menu_top+i == menu_pos) {
			lcd.setCursor(0, i);
			lcd.print("*");
		}
		lcd.setCursor(2, i);
		lcd.print(menu_items[menu_top+i].title);
	}
}

void menu_prev()
{
	if(menu_pos > 0) {
		menu_pos--;
		if(menu_top > menu_pos) {
			menu_top = menu_pos;
		}
	}
}

void menu_next()
{
	if(menu_pos < MI_COUNT-1) {
		menu_pos++;
		if(menu_pos - menu_top >= MENU_LINE_COUNT) {
			menu_top++;
		}
	}
}

void menu_select()
{
	if(menu_items[menu_pos].select_fn) {
		menu_items[menu_pos].select_fn();
	}
}

void on_show_process()
{
	Serial.println("Show Process");
	dmode = DM_STATE;
	update_display();
}

void on_show_ip()
{
	Serial.println("Show IP");
	dmode = DM_NETWORK;
	update_display();
}

void on_show_wifi()
{
	Serial.println("Show Wifi");
	dmode = DM_WIFI;
	update_display();
}

// on change
void rotate(ESPRotary& r)
{
	 Serial.println(r.getPosition());
}

// on left or right rotation
void showDirection(ESPRotary& r)
{
	Serial.println(r.directionToString(r.getDirection()));
	switch(dmode)
	{
		case DM_MENU:
			if(r.getDirection() == RE_RIGHT) {
				menu_prev();
			} else {
				menu_next();
			}
			update_display();
			break;
		case DM_STATE:
			if(r.getDirection() == RE_RIGHT) {
				if(target<MAX_TEMP) {
					target++;
				}
			} else {
				if(target>MIN_TEMP) {
					target--;
				}
			}
			update_display();
			break;
		default:
			break;
	}
}
 
// single click
void click(Button2& btn)
{
	Serial.println("Click!");
	if(dmode == DM_MENU) {
		// menu select
		Serial.println("Select item");
		menu_select();
	} else {
		dmode = DM_MENU;
	}
	update_display();
}

// long click
void resetPosition(Button2& btn)
{
	Serial.println("Reset!");
//	rotary.resetPosition();
	if(dmode == DM_MENU) {
		// reset and show menu
		menu_top = 0;
		menu_pos = 0;
		update_display();
	}
}

void update_display()
{
	switch(dmode) {
		case DM_STATE:
Serial.println("display state");
			display_state();
			break;
		case DM_MENU:
Serial.println("display menu");
			menu_render();
			break;
		case DM_NETWORK:
			display_network();
			break;
		case DM_WIFI:
			display_wifi();
			break;
	}
}

void display_network()
{
	lcd.clear();
	lcd.home();
	lcd.print(WiFi.localIP());
	lcd.setCursor(0, 1);
	lcd.print(wifi_station_get_hostname());
}

void display_wifi()
{
	lcd.clear();
	lcd.home();
	const char* const modes[] = { "NULL", "STA", "AP", "STA+AP" };
	lcd.print(WiFi.SSID());
	lcd.setCursor(0, 1);
	lcd.print("Mode: ");
	lcd.print(modes[wifi_get_opmode()]);
}

void display_state()
{
	if(dmode == DM_STATE) {
		lcd.home();
		if(temp == UNKNOWN_TEMP) {
			lcd.clear();
			lcd.print("Waiting for sensor...");
		} else {
			lcd.print("T: ");
			lcd.print(temp);
			lcd.print(F(" => "));
			lcd.print(target);
			lcd.print(F(" C"));
			lcd.setCursor(0, 1);
			lcd.print("Up: ");
			lcd.print((int)secondary_temp);
			lcd.print(" Heat: ");
			lcd.print(heater);
			if(heater > 0) {
				lcd.print("^");
			}
			lcd.print("  ");
		}
	}
}

void print_status()
{
	if(temp != UNKNOWN_TEMP) {
		Serial.print("\n\nHeating val: ");
		Serial.print(heater);
		Serial.print(", ");
		Serial.print(temp);
		Serial.print(F(" > "));
		Serial.print(target);
		Serial.print(", Humidity: ");
//		Serial.println(am2320.getHumidity());
		Serial.print(si.getHumidityPercent());
		Serial.print(", Top temp:");
		Serial.println(secondary_temp);
	}
}

void pid_compute()
{
	Input = temp;
	Setpoint = target;
	heaterPID.Compute();
//	Serial.print("In:");
//	Serial.print(Input);
//	Serial.print(" Out:");
//	Serial.print(Output);
//	Serial.print(" Set:");
//	Serial.println(Setpoint);
	heater = Output;
//	heater = max_heat_pwm;
	setPwm(HEATER_PIN, heater);
	setPwm(FAN_PIN, max_fan_pwm);
}

float read_primary_temp()
{
	float rs = 0;
	static bool bmp_found = true;
	char result = bmp.startMeasurment();
	if(result!=0){
		delay(result);
		double T, P;
		result = bmp.getTemperatureAndPressure(T,P);
		if(result!=0)
		{
			rs = T;
		}
		else
		{
			if(bmp_found) {
				Serial.println("BMP not found");
				bmp_found = false;
			}
		}
	}
	else
	{
		if(bmp_found) {
			Serial.println("BMP not found");
			bmp_found = false;
		}
	}
	return rs;
}

float read_secondary_temp()
{
	float rs = 0;
	static bool si_found = true;
	if(si.sensorExists()) {
		si_found = true;
		rs = si.getCelsiusHundredths()/100.;
	} else {
		if(si_found) {
			Serial.println("SI not found");
			si_found = false;
		}
	}
	return rs;
}

void measurement()
{
	temp = read_primary_temp();
	if(temp > 0)
	{
		pid_compute();
	}
	else
	{
		// error temp reading
		setPwm(HEATER_PIN, 0);
		setPwm(FAN_PIN, max_fan_pwm/2);
	}
	secondary_temp = read_secondary_temp();
}

void history_collector()
{
	if(temp != UNKNOWN_TEMP) {
		if(history_count >= HISTORY_SIZE) {
			for(int i=1; i<HISTORY_SIZE; i++) {
				temp_history[i-1] = temp_history[i];
				heater_history[i-1] = heater_history[i];
				target_history[i-1] = target_history[i];
			}
			history_count--;
		}
		temp_history[history_count] = (uint8_t)temp;
		heater_history[history_count] = (uint8_t)::map(heater, 0, max_heat_pwm, 0, 100);
		target_history[history_count] = (uint8_t)target;
		secondary_temp_history[history_count] = (uint8_t)secondary_temp;
		history_count++;
	}
}

void update_config()
{
	float v;
	int iv;
	if(ESPHTTPServer.load_user_config("kp", v))
		Kp = v;
	else
		ESPHTTPServer.save_user_config("kp", (float)Kp);
	if(ESPHTTPServer.load_user_config("ki", v))
		Ki = v;
	else
		ESPHTTPServer.save_user_config("ki", (float)Ki);
	if(ESPHTTPServer.load_user_config("kd", v))
		Kd = v;
	else
		ESPHTTPServer.save_user_config("kd", (float)Kd);
	if(ESPHTTPServer.load_user_config("default_temp", iv))
		default_temp = iv;
	else
		ESPHTTPServer.save_user_config("default_temp", default_temp);
	if(target == 0) {
		target = default_temp;
	}
	heaterPID.SetTunings(Kp, Ki, Kd);
	if(ESPHTTPServer.load_user_config("max_heat_pwm", v))
		max_heat_pwm = v;
	else
		ESPHTTPServer.save_user_config("max_heat_pwm", max_heat_pwm);
	if(ESPHTTPServer.load_user_config("max_fan_pwm", v))
		max_fan_pwm = v;
	else
		ESPHTTPServer.save_user_config("max_fan_pwm", max_fan_pwm);
	//tell the PID to range between 0 and max pwm
	heaterPID.SetOutputLimits(0, max_heat_pwm);
	if(!ESPHTTPServer.load_user_config("lcd_on", iv)) {
		iv = BACKLIGHT_ON;
		ESPHTTPServer.save_user_config("lcd_on", BACKLIGHT_ON);
	}
	lcd.setBacklight(iv ? 255 : 0); //0-1
}

void led_blink()
{
	setPwm(LED_BUILTIN, 97);
	delay(30);
	digitalWrite(LED_BUILTIN, HIGH);
}

void setPwm(int pin, int val)
{
	analogWrite(pin, val);
}

void setupPwm()
{
	pinMode(HEATER_PIN, OUTPUT);
	setPwm(HEATER_PIN, 0);
	pinMode(FAN_PIN, OUTPUT);
	setPwm(FAN_PIN, 0);
}

Task measurement_timer(100, TASK_FOREVER, &measurement);
Task history_timer(6000, TASK_FOREVER, &history_collector);
Task display_timer(1000, TASK_FOREVER, &display_state);
Task print_timer(10000, TASK_FOREVER, &print_status);
Task heart_beat(1000, TASK_FOREVER, &led_blink);
Task read_and_update_config(1000, TASK_FOREVER, &update_config);

Scheduler runner;

#define NEW_TARGET_PARAM "new_target"

String history2json(uint8_t h[])
{
	String json = "[";
	for(int i=0; i<history_count; i++) {
		if(i>0) {
			json += ",";
		}
		json += String(h[i]);
	}
	json += "]";
	return json;
}

int timer_value()
{
	return (millis() - initial_millis)/1000;
}

void callbackJSON(AsyncWebServerRequest *request)
{
//Serial.print("json:");
//Serial.println(request->url());

	if (request->url() == "/json/now")
	{
		if(request->hasParam(NEW_TARGET_PARAM)) {
			target = String(request->getParam(NEW_TARGET_PARAM)->value()).toInt();
		}
		String json = "{";
		json += "\"temp\":" + String(temp > 0 ? temp : 0) + ",";
		json += "\"heater\":" + String(::map(heater, 0, max_heat_pwm, 0, 100)) + ",";
		json += "\"target\":" + String(target) + ",";
		json += "\"timer_value\":" + String(timer_value()) + ",";
		json += "\"secondary_temp\":" + String(secondary_temp);
		json += "}";
		request->send(200, "text/json", json);
	} else if (request->url() == "/json/history") {
		String json = "{";
		json += "\"temp\":" + history2json(temp_history) + ",";
		json += "\"heater\":" + history2json(heater_history) + ",";
		json += "\"target\":" + history2json(target_history) + ",";
		json += "\"secondary_temp\":" + history2json(secondary_temp_history);
		json += "}";
		request->send(200, "text/json", json);
	} else if (request->url() == "/json/reset_timer") {
		initial_millis = millis();
		request->send(200, "text/json", "{\"timer_value\": 0}");
	}
}

void  callbackUSERVERSION(AsyncWebServerRequest *request)
{
	String values = VERSION;
	request->send(200, "text/plain", values);
	values = "";
}

void setup()
{
	// setup pwm range 0-100
	analogWriteRange(100);
	Serial.begin(115200);
	Serial.println();
	Serial.setDebugOutput(true);
	Serial.println("Incubator...");

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); // turn on

	setupPwm();

	rotary.setChangedHandler(rotate);
	rotary.setLeftRotationHandler(showDirection);
	rotary.setRightRotationHandler(showDirection);

	button.setTapHandler(click);
	button.setDebounceTime(100);
	button.setLongClickTime(1000);
	button.setLongClickHandler(resetPosition);

	si.begin();
	if(!bmp.begin())
	{
		Serial.println("BMP init failed!");
	}

	Wire.begin();
	Wire.beginTransmission(0x27);
	int error = Wire.endTransmission();
	if (error == 0) {
		Serial.println("LCD found.");
		lcd.begin(16, 2);
	} else {
		Serial.print("LCD not found. Error: ");
		Serial.println(error);
	}

	lcd.begin(16,2);
	lcd.setBacklight(0); //0-1
	lcd.clear();
	lcd.home();
	lcd.print("Thermo");
	lcd.setCursor(0, 1);
	lcd.print(" chambre");

	// WiFi is started inside library
	LittleFS.begin(); // Not really needed, checked inside library and started if needed
	ESPHTTPServer.begin(&LittleFS);

	//set json callback
	ESPHTTPServer.setJSONCallback(callbackJSON);

	//set callback for user version
	ESPHTTPServer.setUSERVERSION(VERSION);

	update_config();

	//tell the PID to range between 0 and max pwm
	heaterPID.SetOutputLimits(0, max_heat_pwm);
	//turn the PID on
	heaterPID.SetMode(AUTOMATIC);
	heaterPID.SetSampleTime(10);

	runner.init();
	runner.addTask(display_timer);
	runner.addTask(measurement_timer);
	runner.addTask(print_timer);
	runner.addTask(heart_beat);
	runner.addTask(history_timer);
	runner.addTask(read_and_update_config);

	display_timer.enable();
	measurement_timer.enable();
	print_timer.enable();
	heart_beat.enable();
	history_timer.enable();
	read_and_update_config.enable();

//	Debug.begin(wifi_station_get_hostname());
//	Debug.setResetCmdEnabled(true); // Enable the reset command
//	Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
//	Debug.showColors(true); // Colors

//	Serial.println("* Arduino RemoteDebug Library");
//	Serial.println("*");
	Serial.print("* WiFI connected. IP address: ");
	Serial.print(WiFi.localIP());
	Serial.print(", Hostname: ");
	Serial.println(wifi_station_get_hostname());

	update_display();

}

void loop()
{
	rotary.loop();
	button.loop();

	runner.execute();
	// DO NOT REMOVE. Attend OTA update from Arduino IDE
//	ESPHTTPServer.handle();
//	Debug.handle();
//	yield();
}
