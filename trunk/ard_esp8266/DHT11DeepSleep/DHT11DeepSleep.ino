#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#define DHTTYPE DHT11
#define DHTPIN	12

const char *thingspeak_server = "184.106.153.149";

#include "config.h"
const char *apiKey = API_KEY;

const char* ssid	 = SSID;
const char* password = PASSWORD;

int ledPin = 5; // LED is attached to ESP8266 pin 5.
int ledState = HIGH;

// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.	For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266


void setup() 
{
	Serial.begin(115200);
	Serial1.begin(115200);

	// Connect to WiFi network
	WiFi.begin(ssid, password);
	Serial1.print("\n\r \n\rWorking to connect");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial1.print(".");
	}
	Serial1.println("");
	Serial1.println("DHT Weather Reading Server");
	Serial1.print("Connected to ");
	Serial1.println(ssid);
	Serial1.print("IP address: ");
	Serial1.println(WiFi.localIP());

	dht.begin();		   // initialize temperature sensor

	pinMode(ledPin, OUTPUT); // Set LED pin (5) as an output
	digitalWrite(ledPin, LOW);
	delay(3000);

}

void loop() 
{
	digitalWrite(ledPin, ledState);
	ledState = !ledState;

	WiFiClient client;
	if (client.connect(thingspeak_server,80)) {  //   "184.106.153.149" or api.thingspeak.com

		// Reading temperature for humidity takes about 250 milliseconds!
		// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
		float humidity = dht.readHumidity();		  // Read humidity (percent)
		float temp_c = dht.readTemperature(false);	   // Read temperature as Fahrenheit
		if(isnan(humidity) || isnan(temp_c)) {
			Serial1.println("Failed to read from DHT sensor!");
			delay(500);
		} else {
			Serial1.print("T:");
			Serial1.print(temp_c);
			Serial1.print(", H:");
			Serial1.println(humidity);

			String postStr = String(apiKey) +
				"&field1=" + String(temp_c) +
				"&field2=" + String(humidity) +
				"\r\n\r\n";

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

			Serial1.println("Data send to Thingspeak");
			ESP.deepSleep(300000000L, WAKE_RF_DEFAULT); // Sleep for 300 seconds
		}
	} else {
		Serial1.println("Http connection failed");
		delay(500);
	}
}