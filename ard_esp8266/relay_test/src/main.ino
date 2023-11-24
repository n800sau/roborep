#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ESP8266Ping.h>

#include "config.h"

#define HOSTNAME "multicast_test"

const char* ssid	 = SSID;
const char* password = PASSWORD;

const int RELAY_PIN = 0;

void setup ()
{
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	Serial.println("");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	if (MDNS.begin(HOSTNAME)) {
		Serial.println("MDNS responder started");
	}

	pinMode(RELAY_PIN, OUTPUT);
	digitalWrite(RELAY_PIN, HIGH);

}

bool relay_state = true;

void  loop ( )
{
	digitalWrite(RELAY_PIN, relay_state);
	relay_state = !relay_state;
	delay(1000);
}
