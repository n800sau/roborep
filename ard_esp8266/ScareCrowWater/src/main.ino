#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>

#include "config.h"

const char* ssid	 = SSID;
const char* password = PASSWORD;

#define PUMP_PIN 4

#define LED_PIN 2


Ticker blinker;
Ticker pumper;

ESP8266WebServer server(80);

volatile int blinking_times = 0;
volatile int blinking_state = 0;


// attention! delays in tickers crash
void blinking()
{
	if(blinking_times > 0) {
		if(blinking_state) {
			Serial.println("led off");
			digitalWrite(LED_PIN, LOW);
			blinking_times--;
		} else {
			Serial.println("led on");
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


void setup()
{
	Serial.begin(115200);
	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);

	// Connect to WiFi network
	WiFi.begin(ssid, password);
	Serial.print("\n\r \n\rWorking to connect");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.println("IP addresses: ");
	Serial.println(WiFi.localIP());

	blink(1);

	if (!MDNS.begin("scarecrowwater")) {
		Serial.println("Error setting up mDNS responder!");
	} else {
		Serial.println("mDNS responder started");
		MDNS.addService("http", "tcp", 80);
	}

	server.on("/", handleRoot);
	server.on("/scare", HTTP_GET, handleScare);
	server.begin();
	Serial.println("HTTP server started");

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
		Serial.print("Stage ");
		Serial.println(pump_stage);
		pump_stage++;
		if(pump_stage >= 2) {
			pump_stage = 0;
			n_loops--;
		}
	}
}

void start_pump(int n)
{
	Serial.print("Pump ");
	Serial.print(n);
	Serial.println(" times");
	pump_stage = 0;
	n_loops = n;
}

void loop()
{
	server.handleClient();
	delay(1);
}
