#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ESP8266Ping.h>

#include "config.h"

#define HOSTNAME "multicast_test"

const char* ssid     = SSID;
const char* password = PASSWORD;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 12345;      // local port to listen on

void  setup ( )
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

}

const IPAddress remote_ip(192, 168, 1, 1);

void  loop ( )
{
	// send no data message
	Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
	Udp.print("Multicast test");
	Udp.endPacket();
	Serial.println("Test");
	delay(1000);
	if(Ping.ping(remote_ip)) {
		Serial.println("Ping success!!");
	} else {
		Serial.println("Ping error.");
	}
	delay(500);
}
