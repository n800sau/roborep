#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <Servo.h> 

#include "config.h"

const char* ssid	 = SSID;
const char* password = PASSWORD;

#define PAN_SERVO_PIN   12
#define TILT_SERVO_PIN  14

#define PUMP_PIN 4

#define LED_PIN 2

#define DBG_SERIAL Serial

Ticker blinker;
Ticker pumper;

ESP8266WebServer server(80);

volatile int blinking_times = 0;
volatile int blinking_state = 0;

Servo pan_servo;  // create servo object to control a servo 
Servo tilt_servo; // twelve servo objects can be created on most boards


// attention! delays in tickers crash
void blinking()
{
	if(blinking_times > 0) {
		if(blinking_state) {
			DBG_SERIAL.println("led off");
			digitalWrite(LED_PIN, LOW);
			blinking_times--;
		} else {
			DBG_SERIAL.println("led on");
			digitalWrite(LED_PIN, HIGH);
		}
		blinking_state = !blinking_state;
	}
}

void blink(int times=1)
{
	blinking_times = times;
	blinking_state = 0;
}

void handleRoot() {
	server.send(200, "text/plain", "hello from " SSID "!");
	blink(1);
}

void handleScare()
{
	start_pump(1);
	server.send(200, "text/plain", "Ok");
	blink(1);
}

void handlePan()
{
	String p = server.arg("pos");
	int pos = (p == "") ? 90 : p.toInt();
	if(pos < 0) {
		pos = 0;
	} else if(pos > 180) {
		pos = 180;
	}
	pan_servo.write(pos);
	server.send(200, "text/plain", "Ok");
	blink(1);
}

void handleTilt()
{
	String p = server.arg("pos");
	int pos = (p == "") ? 90 : p.toInt();
	if(pos < 0) {
		pos = 0;
	} else if(pos > 180) {
		pos = 180;
	}
	tilt_servo.write(pos);
	server.send(200, "text/plain", "Ok");
	blink(1);
}

void setup()
{
	DBG_SERIAL.begin(115200, SERIAL_8N1, SERIAL_FULL);
	DBG_SERIAL.setDebugOutput(true);
	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);

	// Connect to WiFi network
	WiFi.begin(ssid, password);
	DBG_SERIAL.print("\n\r \n\rWorking to connect");

	pan_servo.attach(PAN_SERVO_PIN);
	pan_servo.attach(TILT_SERVO_PIN);

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		DBG_SERIAL.print(".");
	}
	DBG_SERIAL.println("");
	DBG_SERIAL.print("Connected to ");
	DBG_SERIAL.println(ssid);
	DBG_SERIAL.println("IP addresses: ");
	DBG_SERIAL.println(WiFi.localIP());

	blink(1);

	if (!MDNS.begin("scarecrowwater")) {
		DBG_SERIAL.println("Error setting up mDNS responder!");
	} else {
		DBG_SERIAL.println("mDNS responder started");
		MDNS.addService("http", "tcp", 80);
	}

	server.on("/", handleRoot);
	server.on("/scare", HTTP_GET, handleScare);
	server.on("/tilt", HTTP_GET, handleTilt);
	server.on("/pan", HTTP_GET, handlePan);
	server.begin();
	DBG_SERIAL.println("HTTP server started");

	pumper.attach(1, pumping);
	blinker.attach(0.3, blinking);
}

volatile int pump_stage = 0;
volatile int n_loops = 0;

void pumping()
{
	if(n_loops > 0) {
		switch(pump_stage) {
			case 0:
				digitalWrite(PUMP_PIN, HIGH);
				break;
			case 1:
				digitalWrite(PUMP_PIN, LOW);
				break;
		}
		DBG_SERIAL.print("Stage ");
		DBG_SERIAL.println(pump_stage);
		pump_stage++;
		if(pump_stage >= 2) {
			pump_stage = 0;
			n_loops--;
		}
	}
}

void start_pump(int n)
{
	DBG_SERIAL.print("Pump ");
	DBG_SERIAL.print(n);
	DBG_SERIAL.println(" times");
	pump_stage = 0;
	n_loops = n;
}

void loop()
{
	server.handleClient();
	delay(1);
}
