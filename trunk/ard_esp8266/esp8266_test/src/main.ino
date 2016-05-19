#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "nec_codes.h"

#include "config.h"

#define HOSTNAME "udp2led"

const char* ssid     = SSID;
const char* password = PASSWORD;

boolean wifiConnected = false;

// UDP variables
unsigned int localPort = 8888;

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 12345;      // local port to listen on


WiFiUDP UDP;
boolean udpConnected = false;

char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet

// strings to send back
const char replyAck[] = "ack"; 
const char replyNak[] = "nak"; 

const int RED_PIN = 4;
const int GREEN_PIN = 12;
const int YELLOW_PIN = 13;
const int BLUE_PIN = 14;

void setup()
{
	Serial.begin(115200);
	// initialise pins
	pinMode(RED_PIN, OUTPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(YELLOW_PIN, OUTPUT);
	pinMode(BLUE_PIN, OUTPUT);

}

void blink(int pin, int times=5)
{
	Serial.print("BLINK ");
	Serial.println(pin);
	for(int i=0; i<times; i++) {
		digitalWrite(pin, HIGH);
		delay(100);
		digitalWrite(pin, LOW);
		if(i < times) {
			delay(200);
		}
	}
}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(wifiConnected)
	{
		if(udpConnected)
		{
			// if there’s data available, read a packet
			int packetSize = UDP.parsePacket();
			if(packetSize)
			{
				Serial.println("");
				Serial.print("Received packet of size ");
				Serial.println(packetSize);
				Serial.print("From ");
				IPAddress remote = UDP.remoteIP();
				for (int i =0; i < 4; i++)
				{
					Serial.print(remote[i], DEC);
					if (i < 3)
					{
						Serial.print(".");
					}
				}
				Serial.print(", port ");
				Serial.println(UDP.remotePort());

				// read the packet into packetBufffer
				UDP.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
				Serial.println("Contents:");
				packetBuffer[packetSize] = 0;
				Serial.println(packetBuffer);

				StaticJsonBuffer<200> jsonBuffer;
				JsonObject& json = jsonBuffer.parseObject(packetBuffer);

				// send a reply, to the IP address and port that sent us the packet we received
				UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());

				if (json.success()) {
					UDP.write(replyAck);
					const char* encoding = json["encoding"];
					if(encoding && strcmp(encoding, MODEL) == 0) {
						int ircode = json["ircode"];
						Serial.println(encoding);
						Serial.print("0x");
						Serial.println(ircode, HEX);
						switch(ircode) {
							case IR_RED:
								blink(RED_PIN, 3);
								break;
							case IR_GREEN:
								blink(GREEN_PIN, 2);
								break;
							case IR_YELLOW:
								blink(YELLOW_PIN, 1);
								break;
							case IR_BLUE:
								blink(BLUE_PIN, 5);
								break;
						}
					} else {
						Serial.println("Unknown encoding");
					}
				} else {
					UDP.write(replyNak);
					Serial.println("Failed to parse config file");
				}

				UDP.endPacket();

			}
		} else {
			udpConnected = connectUDP();
		}
	} else {
		wifiConnected = connectWifi();
		if(wifiConnected) {
			if(MDNS.begin(HOSTNAME)) {
				Serial.println("MDNS responder " HOSTNAME " started");
			}
		}
	}
	delay(10);
}

	// connect to UDP – returns true if successful or false if not
boolean connectUDP()
{
	boolean state = false;

	Serial.println("");
	Serial.println("Connecting to UDP");

	if(UDP.beginMulticast(WiFi.localIP(), ipMulti, portMulti) == 1) {
		Serial.println("Connection successful");
		state = true;
	} else {
		Serial.println("Connection failed");
	}

	return state;
}

	// connect to wifi – returns true if successful or false if not
boolean connectWifi()
{
	boolean state = true;
	int i = 0;
	WiFi.begin(ssid, password);
	Serial.println("");
	Serial.println("Connecting to WiFi");

	// Wait for connection
	Serial.print("Connecting");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
		if (i > 10){
			state = false;
			break;
		}
		i++;
	}
	if (state)
	{
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(ssid);
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
		Serial.println("");
		Serial.println("Connection failed.");
	}
	return state;
}

