/*

am2320
sda - A4
scl - A5

heater pwm - 3

*/

#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ticker.h>

#define OLED_ADDRESS 0x3c
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "local_config.h"

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

/* Start Webserver */
AsyncWebServer webserver(80);
//AsyncWebSocket ws("/ws");
//AsyncEventSource events("/events");

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

#include <AM2320.h>
AM2320 am2320;


#if defined(ESP8266)
const int HEATER_PIN = D5;
const int FAN_PIN = D6;
#else // arduino
const int HEATER_PIN = 3;
const int FAN_PIN = 5;
#endif

// 80 - 8v
const int MAX_HEAT_PWM = 80;
const int MAX_FAN_PWM = 200;

int heat_val = MAX_HEAT_PWM;

int temp2set = 37;

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=80, Ki=0, Kd=10;
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;

void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}

void display_status()
{
	if(temp != UNKNOWN_TEMP) {
		display.clearDisplay();
		display.setCursor(18, 0);
		display.print("T: ");
		display.print(temp);
		display.print(F(" => "));
		display.print(temp2set);
		display.print(F(" C"));
		display.setCursor(18, 11);
		display.print("Heating: ");
		display.print(heat_val);
		if(heat_val > 0) {
			display.fillTriangle(2, 18, 10, 18, 6, 10, INVERSE);
		}
		display.display();
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
		Serial.println(am2320.getHumidity());
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
	if (am2320.measure()) {
		temp = am2320.getTemperature();
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
		heat_val = MAX_HEAT_PWM;
		analogWrite(HEATER_PIN, heat_val);
		analogWrite(FAN_PIN, MAX_FAN_PWM);
	}
	else
	{
		analogWrite(HEATER_PIN, 0);
		analogWrite(FAN_PIN, 0);
		int errorCode = am2320.getErrorCode();
		switch (errorCode)
		{
			case 1: Serial.println("ERR: am2320 is offline"); break;
			case 2: Serial.println("ERR: CRC validation failed."); break;
		}
	}
}

Ticker measurement_timer(measurement, 100, 0, MILLIS);
Ticker display_timer(display_status, 1000, 0, MILLIS);
Ticker print_timer(print_status, 10000, 0, MILLIS);
Ticker chart_timer(update_chart_data, 1000 * T_DATA_COLLECTION_STEP, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	Serial.println();
	Serial.setDebugOutput(true);
	Serial.println("Incubator...");
	pinMode(HEATER_PIN, OUTPUT);
	pinMode(FAN_PIN, OUTPUT);
	memset(&t_data, 0, sizeof(t_data));
	am2320.begin();
	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { // Address 0x3D for 128x64
		Serial.println(F("SSD1306 allocation failed"));
	} else {
		display.clearDisplay();
		display.display();
		// text display tests
		display.setTextSize(1);
		display.setTextColor(WHITE);
		display.setCursor(30, 10);
		display.print(F("I am a heater"));
		display.display();

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

		display_timer.start();
		chart_timer.start();
		measurement_timer.start();
		print_timer.start();

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
}
void loop()
{
	measurement_timer.update();
	display_timer.update();
	chart_timer.update();
	print_timer.update();
}
