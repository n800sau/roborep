/*

am2320
sda - A4
scl - A5

heater pwm - 3

*/

#include <PID_v1.h>
#include <TaskScheduler.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <SoftwareSerial.h>

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <TimeLib.h>
//#include <NtpClientLib.h>
//#include <ArduinoOTA.h>
//#include <ArduinoJson.h>
//#include <FSWebServerLib.h>
//#include <Hash.h>

#include "local_config.h"

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

/* Start Webserver */
AsyncWebServer webserver(80);

#include <ESPDash.h>

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&webserver);
// Bar Chart Instance
Chart t_chart(&dashboard, BAR_CHART, "Temperature Plot");
Card slider(&dashboard, SLIDER_CARD, "Temperature", "", 20, 50);
Card t(&dashboard, TEMPERATURE_CARD, "Temperature Now", "°C");
// Bar Chart Data
#define X_COUNT 7
String XAxis[X_COUNT] = {"-6m", "-5m", "-4m", "-3m", "-2m", "-1m", "Now"};
int YAxis[X_COUNT] = {0, 0, 0, 0, 0, 0, 0};
// data collected once in a T_DATA_COLLECTION_STEP (seconds)
#define T_DATA_COLLECTION_STEP 10
#define T_DATA_COUNT (X_COUNT * 60 / T_DATA_COLLECTION_STEP)
int t_data[T_DATA_COUNT];

#include <SI7021.h>
SI7021 si;

//#include <AM2320.h>
//AM2320 am2320;


#if defined(ESP8266)
const int HEATER_PIN = -100;
const int FAN_PIN = -200;
//const int HEATER_PIN = D5;
//const int FAN_PIN = D6;
const int ROTARY_KEY = D7;
const int ROTARY_S1 = D5;
const int ROTARY_S2 = D6;
const int RX_PIN = D3;
const int TX_PIN = -1;
#else // arduino
const int HEATER_PIN = 3;
const int FAN_PIN = 5;
#endif

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
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;

SoftwareSerial swSer;

void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}

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

void update_chart_data()
{
	if(temp != UNKNOWN_TEMP) {
		for(int i=0; i<T_DATA_COUNT-1;i++) {
			t_data[i] = t_data[i+1];
		}
		t_data[T_DATA_COUNT-1] = temp;
		for(int i=0; i<X_COUNT; i++) {
			YAxis[i] = 0;
			int v_count = 0;
			for(int j=0; j < (60/T_DATA_COLLECTION_STEP); j++) {
				int v = t_data[i*60/T_DATA_COLLECTION_STEP+j];
				if(v > 0) {
					YAxis[i] += v;
					v_count++;
				}
			}
			if(v_count > 0) {
				YAxis[i] /= v_count;
			}
		}
		/* Update Chart Y Axis (yaxis_array, array_size) */
		t_chart.updateY(YAxis, X_COUNT);
		/* Send Updates to our Dashboard (realtime) */
		dashboard.sendUpdates();
	}
}

void measurement()
{
	if(si.sensorExists()) {
//	if (am2320.measure()) {
//		temp = am2320.getTemperature();
		temp = si.getCelsiusHundredths()/100.;
		t.update(temp);
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
//		int errorCode = am2320.getErrorCode();
//		switch (errorCode)
//		{
//			case 1: Serial.println("ERR: am2320 is offline"); break;
//			case 2: Serial.println("ERR: CRC validation failed."); break;
//		}
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
Task chart_timer(1000 * T_DATA_COLLECTION_STEP, TASK_FOREVER, &update_chart_data);

//Ticker measurement_timer(measurement, 100, 0, MILLIS);
//Ticker display_timer(display_status, 1000, 0, MILLIS);
//Ticker print_timer(print_status, 10000, 0, MILLIS);
//Ticker chart_timer(update_chart_data, 1000 * T_DATA_COLLECTION_STEP, 0, MILLIS);

Scheduler runner;

void setup()
{
	Serial.begin(115200);
	Serial.println();
	Serial.setDebugOutput(true);
	Serial.println("Incubator...");
	setupPwm();
	memset(&t_data, 0, sizeof(t_data));

    // WiFi is started inside library
//    SPIFFS.begin(); // Not really needed, checked inside library and started if needed
//    ESPHTTPServer.begin(&SPIFFS);

//	am2320.begin();
	si.begin();
  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
//  Wire.begin();
  Wire.beginTransmission(0x27);
  int error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    lcd.begin(16, 2); // initialize the lcd

  } else {
    Serial.println(": LCD not found.");
  }

	lcd.begin(16,2);
    lcd.setBacklight(255);
	lcd.clear();
	lcd.home();
	lcd.print("Thermo");
	lcd.setCursor(0, 1);
	lcd.print(" chambre");

	//tell the PID to range between 0 and max pwm
	heaterPID.SetOutputLimits(0, MAX_HEAT_PWM);

	//turn the PID on
	heaterPID.SetMode(AUTOMATIC);
	heaterPID.SetSampleTime(10);

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());

	runner.init();
	runner.addTask(display_timer);
	runner.addTask(chart_timer);
	runner.addTask(measurement_timer);
	runner.addTask(print_timer);

	display_timer.enable();
	chart_timer.enable();
	measurement_timer.enable();
	print_timer.enable();

//	display_timer.start();
//	chart_timer.start();
//	measurement_timer.start();
//	print_timer.start();

	t_chart.updateX(XAxis, X_COUNT);
	slider.update(temp2set);
	/* Attach Slider Callback */
	slider.attachCallback([&](int value){
		/* Print our new slider value received from dashboard */
		Serial.println("Slider Triggered: "+String(value));
		temp2set = value;
		/* Make sure we update our slider's value and send update to dashboard */
		slider.update(value);
		dashboard.sendUpdates();
	});

	webserver.onNotFound(notFound);
	/* Start AsyncWebServer */
	webserver.begin();

}
void loop()
{
	runner.execute();
//	measurement_timer.update();
//	display_timer.update();
//	chart_timer.update();
//	print_timer.update();
//	ESPHTTPServer.handle();
}
