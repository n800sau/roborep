#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>

#include "config.h"

const char* ssid	 = SSID;
const char* password = PASSWORD;

const char *ap_ssid = AP_SSID;
const char *ap_password = AP_PASSWORD;

#define MOTO_PIN_1 4
#define MOTO_PIN_2 5

#define LED_PIN 2


Ticker blinker;
Ticker switcher;

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
	server.send(200, "text/plain", "hello from " AP_SSID "!");
	blink(1);
}

void handleScare()
{
	String N = server.arg("N");
	if(N == "") {
		start_motors(1);
	} else {
		start_motors(N.toInt());
	}
	server.send(200, "text/plain", "Ok");
	blink(1);
}


void setup()
{
	Serial.begin(115200);
	pinMode(MOTO_PIN_1, OUTPUT);
	pinMode(MOTO_PIN_2, OUTPUT);
	digitalWrite(MOTO_PIN_1, LOW);
	digitalWrite(MOTO_PIN_2, LOW);

	Serial.println(F("\n" AP_SSID));

	WiFi.softAP(ap_ssid, ap_password, 9);
	WiFi.softAPConfig(IPAddress(192, 168, 5, 1), IPAddress(192, 168, 5, 1), IPAddress(192, 168, 5, 0));

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
	Serial.println(WiFi.softAPIP());

	blink(1);

	if (!MDNS.begin("scarecrow")) {
		Serial.println("Error setting up mDNS responder!");
	} else {
		Serial.println("mDNS responder started");
		MDNS.addService("http", "tcp", 80);
	}

	server.on("/", handleRoot);
	server.on("/scare", HTTP_GET, handleScare);
	server.begin();
	Serial.println("HTTP server started");

	switcher.attach(1, switch_motors);
	blinker.attach(0.3, blinking);
}

volatile int stage = 0;
volatile int n_loops = 0;

void switch_motors()
{
	if(n_loops > 0) {
		switch(stage) {
			case 0:
				digitalWrite(MOTO_PIN_1, LOW);
				digitalWrite(MOTO_PIN_2, LOW);
				break;
			case 1:
				digitalWrite(MOTO_PIN_1, HIGH);
				digitalWrite(MOTO_PIN_2, LOW);
				break;
			case 2:
				digitalWrite(MOTO_PIN_1, LOW);
				digitalWrite(MOTO_PIN_2, LOW);
				break;
			case 3:
				digitalWrite(MOTO_PIN_1, LOW);
				digitalWrite(MOTO_PIN_2, HIGH);
				break;
		}
		Serial.print("Stage ");
		Serial.println(stage);
		stage++;
		if(stage >= 4) {
			stage = 0;	
			n_loops--;
		}
	}
}

void start_motors(int n)
{
	Serial.print("Switch motors ");
	Serial.print(n);
	Serial.println(" times");
	stage = 0;
	n_loops = n;
}

void loop()
{
	server.handleClient();
	delay(1);
}
