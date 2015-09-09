// get data from DHT11 nrf pipe and send it to thingspeak

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <SPI.h>
#include "RF24.h"
#include<stdlib.h>

const char *thingspeak_server = "184.106.153.149";

#include "config.h"
const char *apiKey = API_KEY;

const char* ssid	 = SSID;
const char* password = PASSWORD;

#define LED_PIN 4

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(5,15);

/**********************************************************/

const int DTH11_pipe = 1;
const int RF24g_pipe = 2;

byte myaddress[6] = "ESPma";

byte addresses[][6] = {"DHT11", "RF24g" };

void blink(int times=1)
{
	for(int i=0; i<times; i++) {
		digitalWrite(LED_PIN, HIGH);
		delay(100);
		digitalWrite(LED_PIN, LOW);
		if(i < times) {
			delay(200);
		}
	}
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	Serial.begin(115200);
	Serial.println(F("\nRF24 gate"));

	// Connect to WiFi network
	WiFi.begin(ssid, password);
	Serial.print("\n\r \n\rWorking to connect");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("DHT Weather Reading Server");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	blink(1);

	radio.begin();

	// Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	radio.setPALevel(RF24_PA_MAX);
	
	radio.openWritingPipe(myaddress);
	radio.openReadingPipe(DTH11_pipe, addresses[0]);
	radio.openReadingPipe(RF24g_pipe, addresses[1]);
	
	// Start the radio listening for data
	radio.startListening();
}

void loop() {
	char buf[32];
	uint8_t pipenum;
	if( radio.available(&pipenum) ){

		while (radio.available()) {
			radio.read(buf, sizeof(buf));
		}
	 
		radio.stopListening();
		radio.write("OK", 3);
		radio.startListening();
		blink();

		Serial.print(F("Pipe "));
		Serial.print(pipenum);
		Serial.println(F(" Sent response "));

		if(pipenum == DTH11_pipe) {
			int pos = 0;
			float t=-1, h=-1, heat=-1, v=-1;
			char intbuf[30];
			while(pos<sizeof(buf) && buf[pos]) {
				switch(buf[pos]) {
					case 'T':
						t = atoi(buf+pos+1) / 1000.;
						break;
					case 'H':
						h = atoi(buf+pos+1) / 1000.;
						break;
					case 'I':
						heat = atoi(buf+pos+1) / 1000.;
						break;
					case 'V':
						v = atoi(buf+pos+1) / 1000.;
						break;
				}
				pos++;
			}
			Serial.print("T:");
			Serial.print(t);
			Serial.print(" H:");
			Serial.print(h);
			Serial.print(" Heat:");
			Serial.print(heat);
			Serial.print(" V:");
			Serial.println(v);
			send_dht11_data(t, h, v);
		}
	}


} // Loop


void send_dht11_data(float temp_c, float humidity, float v)
{
	WiFiClient client;
	if (client.connect(thingspeak_server,80)) {  //   "184.106.153.149" or api.thingspeak.com

		String postStr = String(apiKey) +
			"&field1=" + String(temp_c) +
			"&field2=" + String(humidity) +
			"&field3=" + String(v) + "\r\n\r\n";

		client.print("POST /update HTTP/1.1\n");
		client.print("Host: api.thingspeak.com\n");
		client.print("Connection: close\n");
		client.print("X-THINGSPEAKAPIKEY: "+String(apiKey)+"\n");
		client.print("Content-Type: application/x-www-form-urlencoded\n");
		client.print("Content-Length: ");
		client.print(postStr.length());
		client.print("\n\n");
		client.print(postStr);
		client.stop();

		Serial.println("Data send to Thingspeak:");
		Serial.println(postStr);
	} else {
		blink(2);
		Serial.println("Http connection failed");
	}
}
