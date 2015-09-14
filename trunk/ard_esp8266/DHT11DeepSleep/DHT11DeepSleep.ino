#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#define DHTTYPE DHT11
#define DHTPIN	5

const char *thingspeak_server = "184.106.153.149";

#include "config.h"
const char *apiKey = API_KEY;

const char* ssid	 = SSID;
const char* password = PASSWORD;

int LED_PIN = 4; // LED is attached to ESP8266 pin 5.

// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.	For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

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


void setup() 
{
	Serial.begin(115200);

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

	dht.begin();		   // initialize temperature sensor

	pinMode(LED_PIN, OUTPUT); // Set LED pin (5) as an output
	digitalWrite(LED_PIN, LOW);
	delay(3000);

}

void loop() 
{

	WiFiClient client;
	if (client.connect(thingspeak_server,80)) {  //   "184.106.153.149" or api.thingspeak.com

		// Reading temperature for humidity takes about 250 milliseconds!
		// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
		float humidity = dht.readHumidity();		  // Read humidity (percent)
		float temp_c = dht.readTemperature(false);	   // Read temperature as Fahrenheit
		float v = ESP.getVcc() / 1000.;
		if(isnan(humidity) || isnan(temp_c)) {
			Serial.println("Failed to read from DHT sensor!");
			blink(2);
			delay(500);
		} else {
			blink();
			Serial.print("T:");
			Serial.print(temp_c);
			Serial.print(", H:");
			Serial.println(humidity);

			String postStr = String(apiKey) +
				"&field1=" + String(temp_c) +
				"&field2=" + String(humidity) +
				"&field3=" + String(v) +
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

			Serial.println("Data send to Thingspeak");
			ESP.deepSleep(10000000L, WAKE_RF_DEFAULT); // Sleep for 10 seconds
		}
	} else {
		blink(3);
		Serial.println("Http connection failed");
		delay(500);
	}
}
