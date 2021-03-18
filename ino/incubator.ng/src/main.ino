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

const int HEATER_PIN = -100;
const int FAN_PIN = -200;
//const int HEATER_PIN = D5;
//const int FAN_PIN = D6;
const int ROTARY_KEY = D7;
const int ROTARY_S1 = D5;
const int ROTARY_S2 = D6;
const int RX_PIN = D3;
const int TX_PIN = -1;

// 80 - 8v
const int MAX_HEAT_PWM = 80;
const int MAX_FAN_PWM = 70;

const char *pwm_command_prefix = "Pwm Command";

int heat_val = MAX_HEAT_PWM;

int temp2set = 37;

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=80, Ki=0, Kd=10;

#include <PID_v1.h>
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;

#include <SoftwareSerial.h>
SoftwareSerial swSer;

//#include <RemoteDebug.h>
//RemoteDebug Debug;

void display_status()
{
	if(temp != UNKNOWN_TEMP) {
		lcd.home();
		lcd.print("T: ");
		lcd.print(temp);
		lcd.print(F(" => "));
		lcd.print(temp2set);
		lcd.print(F(" C"));
		lcd.setCursor(0, 1);
		lcd.print("Heating: ");
		lcd.print(heat_val);
		if(heat_val > 0) {
			lcd.print("^");
		}
		lcd.print("  ");
	}
}

void print_status()
{
	if(temp != UNKNOWN_TEMP) {
		Serial.print("\n\nHeating val: ");
		Serial.print(heat_val);
		Serial.print(", ");
		Serial.print(temp);
		Serial.print(F(" > "));
		Serial.print(temp2set);
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
		Setpoint = temp2set;
		heaterPID.Compute();
//		Serial.print("In:");
//		Serial.print(Input);
//		Serial.print(" Out:");
//		Serial.print(Output);
//		Serial.print(" Set:");
//		Serial.println(Setpoint);
		heat_val = Output;
//		heat_val = MAX_HEAT_PWM;
		setPwm(HEATER_PIN, heat_val);
		setPwm(FAN_PIN, MAX_FAN_PWM);
	} else {
		Serial.println("SI not found");
		setPwm(HEATER_PIN, 0);
		setPwm(FAN_PIN, 0);
	}
}

void setPwm(int pin, int val)
{
//	analogWrite(pin, val);
	swSer.flush();
	swSer.enableTx(true);
	swSer.write(pwm_command_prefix, 3);
	swSer.write(pin == HEATER_PIN ? 0x81 : 0x82);
	swSer.write((uint8_t)float(val)/100*255);
	swSer.enableTx(false);

}

void setupPwm()
{
	swSer.begin(115200, SWSERIAL_8N1, RX_PIN, TX_PIN, false, 64);
//	analogWriteRange(100);
//	pinMode(HEATER_PIN, OUTPUT);
	setPwm(HEATER_PIN, 0);
//	pinMode(FAN_PIN, OUTPUT);
	setPwm(FAN_PIN, 0);
}

Task measurement_timer(100, TASK_FOREVER, &measurement);
Task display_timer(1000, TASK_FOREVER, &display_status);
Task print_timer(10000, TASK_FOREVER, &print_status);

Scheduler runner;

void callbackJSON(AsyncWebServerRequest *request)
{
Serial.print("json:");
Serial.println(request->url());
	if (request->url() == "/json/temp")
	{
		String json = "{";
		json += "\"temp\":" + String(temp);
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
	lcd.setBacklight(0);
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

void loop() {

	runner.execute();
	// DO NOT REMOVE. Attend OTA update from Arduino IDE
	ESPHTTPServer.handle();
//	Debug.handle();
//	yield();
}
