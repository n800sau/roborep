#include <ESP8266WiFi.h>
#include <FS.h>
#include <WiFiClient.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <FSWebServerLib.h>
#include <Hash.h>

#include <TaskScheduler.h>

#include <Wire.h>

#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <SI7021.h>
SI7021 si;

#define VERSION "0.1a"

const int HEATER_PIN = D7;
const int FAN_PIN = D8;

const int ROTARY_KEY = D0;
const int ROTARY_S1 = D5;
const int ROTARY_S2 = D6;

const int CLICKS_PER_STEP = 4;   // this number depends on your rotary encoder 

#include "Button2.h"
#include "ESPRotary.h"
ESPRotary rotary = ESPRotary(ROTARY_S1, ROTARY_S2, CLICKS_PER_STEP);
Button2 button = Button2(ROTARY_KEY);

// 80 - 8v
const int MAX_HEAT_PWM = 80;
const int MAX_FAN_PWM = 70;

const char *pwm_command_prefix = "PWM";

int heater = MAX_HEAT_PWM;

int target = 37;

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=80, Ki=0, Kd=10;

#include <PID_v1.h>
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;

//#include <RemoteDebug.h>
//RemoteDebug Debug;

// on change
void rotate(ESPRotary& r)
{
	 Serial.println(r.getPosition());
}

// on left or right rotation
void showDirection(ESPRotary& r)
{
	Serial.println(r.directionToString(r.getDirection()));
}
 
// single click
void click(Button2& btn)
{
	Serial.println("Click!");
}

// long click
void resetPosition(Button2& btn)
{
	rotary.resetPosition();
	Serial.println("Reset!");
}

void display_status()
{
	if(temp != UNKNOWN_TEMP) {
		lcd.home();
		lcd.print("T: ");
		lcd.print(temp);
		lcd.print(F(" => "));
		lcd.print(target);
		lcd.print(F(" C"));
		lcd.setCursor(0, 1);
		lcd.print("Heating: ");
		lcd.print(heater);
		if(heater > 0) {
			lcd.print("^");
		}
		lcd.print("  ");
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
		Serial.println(si.getHumidityPercent());
	}
}

void measurement()
{
	if(si.sensorExists()) {
		temp = si.getCelsiusHundredths()/100.;
		Input = temp;
		Setpoint = target;
		heaterPID.Compute();
//		Serial.print("In:");
//		Serial.print(Input);
//		Serial.print(" Out:");
//		Serial.print(Output);
//		Serial.print(" Set:");
//		Serial.println(Setpoint);
		heater = Output;
//		heater = MAX_HEAT_PWM;
		setPwm(HEATER_PIN, heater);
		setPwm(FAN_PIN, MAX_FAN_PWM);
	} else {
		Serial.println("SI not found");
		setPwm(HEATER_PIN, 0);
		setPwm(FAN_PIN, 50);
	}
}

void setPwm(int pin, int val)
{
	analogWrite(pin, val);
}

void setupPwm()
{
	analogWriteRange(100);
	pinMode(HEATER_PIN, OUTPUT);
	setPwm(HEATER_PIN, 0);
	pinMode(FAN_PIN, OUTPUT);
	setPwm(FAN_PIN, 0);
}

Task measurement_timer(100, TASK_FOREVER, &measurement);
Task display_timer(1000, TASK_FOREVER, &display_status);
Task print_timer(10000, TASK_FOREVER, &print_status);

Scheduler runner;

#define NEW_TARGET_PARAM "new_target"

void callbackJSON(AsyncWebServerRequest *request)
{
Serial.print("json:");
Serial.println(request->url());

	if (request->url() == "/json/temp")
	{
		if(request->hasParam(NEW_TARGET_PARAM)) {
			target = String(request->getParam(NEW_TARGET_PARAM)->value()).toInt();
		}
		String json = "{";
		json += "\"temp\":" + String(temp > 0 ? temp : 0) + ",";
		json += "\"heater\":" + String(::map(heater, 0, MAX_HEAT_PWM, 0, 100)) + ",";
		json += "\"target\":" + String(target);
		json += "}";
		request->send(200, "text/json", json);
	}
}

void  callbackUSERVERSION(AsyncWebServerRequest *request)
{
	String values = VERSION;
	request->send(200, "text/plain", values);
	values = "";
}

void setup() {
	Serial.begin(115200);
	Serial.println();
	Serial.setDebugOutput(true);
	Serial.println("Incubator...");
	setupPwm();

	rotary.setChangedHandler(rotate);
	rotary.setLeftRotationHandler(showDirection);
	rotary.setRightRotationHandler(showDirection);

	button.setTapHandler(click);
	button.setLongClickHandler(resetPosition);

	si.begin();

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
	SPIFFS.begin(); // Not really needed, checked inside library and started if needed
	ESPHTTPServer.begin(&SPIFFS);

	//set optioanl callback
	ESPHTTPServer.setJSONCallback(callbackJSON);

	//set optioanl callback for user version
	ESPHTTPServer.setUSERVERSION(VERSION);

	//tell the PID to range between 0 and max pwm
	heaterPID.SetOutputLimits(0, MAX_HEAT_PWM);
	//turn the PID on
	heaterPID.SetMode(AUTOMATIC);
	heaterPID.SetSampleTime(10);

	runner.init();
	runner.addTask(display_timer);
	runner.addTask(measurement_timer);
	runner.addTask(print_timer);

	display_timer.enable();
	measurement_timer.enable();
	print_timer.enable();

//	Debug.begin(wifi_station_get_hostname());
//	Debug.setResetCmdEnabled(true); // Enable the reset command
//	Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
//	Debug.showColors(true); // Colors

	Serial.println("* Arduino RemoteDebug Library");
	Serial.println("*");
	Serial.print("* WiFI connected. IP address: ");
	Serial.print(WiFi.localIP());
	Serial.print(", Hostname: ");
	Serial.println(wifi_station_get_hostname());

}

void loop()
{
	rotary.loop();
	button.loop();

	runner.execute();
	// DO NOT REMOVE. Attend OTA update from Arduino IDE
	ESPHTTPServer.handle();
//	Debug.handle();
//	yield();
}
