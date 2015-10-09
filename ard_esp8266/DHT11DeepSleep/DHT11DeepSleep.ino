#include "DHT11DeepSleep.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "FS.h"
#include <DHT.h>
#include <Wire.h>
#include <DS1307.h>
#include <Time.h>

#include "config.h"

#define DHTTYPE DHT11
#define DHTPIN	5

const char *master_server = "192.168.4.1";
//const char *master_server = "nrfgate.local";

const char *STORAGE_NAME = "/storage.bin";
// if file system is available
bool fs_ok;

const char* ssid	 = SSID;
const char* password = PASSWORD;

int LED_PIN = 4; // LED is attached to ESP8266 pin 4.

#define SLEEP_SECS 6


// To read VCC voltage
// TOUT pin has to be disconnected in this mode.
ADC_MODE(ADC_VCC);

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

	Wire.begin(12, 14);


	// Connect to WiFi network
	WiFi.begin(ssid, password);
	Serial.print("\n\r \n\rWorking to connect");

	// Wait for connection
	int conn_status;
	do {
		Serial.print(".");
		conn_status = WiFi.status();
		delay(500);
	} while(conn_status != WL_CONNECTED);
	Serial.println("");
	Serial.println("DHT Weather Reader");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	dht.begin();		   // initialize temperature sensor

	pinMode(LED_PIN, OUTPUT); // Set LED pin (5) as an output
	digitalWrite(LED_PIN, LOW);

	setTime(request_time());

	setSyncProvider(request_time);  //set function to call when sync required

	if (timeStatus()!= timeNotSet) {

		Serial.println("Setting time...");
		RTC.stop();
		RTC.set(DS1307_SEC, second());        //set the seconds
		RTC.set(DS1307_MIN, minute());     //set the minutes
		RTC.set(DS1307_HR, hour());       //set the hours
		RTC.set(DS1307_DOW, weekday());       //set the day of the week
		RTC.set(DS1307_DATE, day());       //set the date
		RTC.set(DS1307_MTH, month());        //set the month
		RTC.set(DS1307_YR, year());         //set the year
		RTC.start();
	}

	fs_ok = SPIFFS.begin();
	if(!fs_ok) {
		SPIFFS.format();
		fs_ok = SPIFFS.begin();
	}
	if(!fs_ok) {
		Serial.println("Failed to start filesystem");
		blink(5);
	}

	delay(3000);

}

bool send_sensor_data(DATA_T &data)
{
	WiFiClient client;
	String url = "/dht11";
	bool rs = client.connect(master_server, 80);
	if (rs) {
		String postStr = "T=" + String(data.temp_c) +
			"&H=" + String(data.humidity) +
			"&V=" + String(data.v) +
			"&TIME=" + String(data.timestamp);

		client.print("POST "+url+" HTTP/1.1\n");
		client.print("Host: " + String(master_server) + "\n");
		client.print("Connection: close\n");
		client.print("Content-Type: application/x-www-form-urlencoded\n");
		client.print("Content-Length: ");
		client.print(postStr.length());
		client.print("\r\n\r\n");
		client.print(postStr + "\r\n\r\n");
		client.stop();
		Serial.print(postStr);
		Serial.println(" sent to " + String(master_server));
	} else {
		Serial.println("Http connection failed (\"/epoch\")");
	}
	return rs;
}

time_t request_time()
{
	time_t rs = 0;
	WiFiClient client;
	String url = "/epoch";
	for(int i=0; i<5; i++) {
		if(client.connect(master_server, 80)) {
			client.print("GET "+url+" HTTP/1.1\n");
			client.print("Host: " + String(master_server) + "\n");
			client.print("Accept: */*\n");
			client.print("Connection: close\n");
			client.print("\r\n\r\n");
			// Read all the lines of the reply from server and print them to Serial
			while(client.connected()){
				String line = client.readStringUntil('\r\n\r\n');
				Serial.print("Headers:");
				Serial.println(line);
				line = client.readStringUntil('\r');
				Serial.print("Reply:");
				Serial.println(line);
				rs = line.toInt();
	  		}
			Serial.println("client stop");
			client.stop();
			break;
		} else {
			Serial.println("Http connection failed (\"/epoch\")");
			delay(1000);
		}
	}
	return rs;
}

void loop()
{


  Serial.print(RTC.get(DS1307_HR, true)); //read the hour and also update all the values by pushing in true
  Serial.print(":");
  Serial.print(RTC.get(DS1307_MIN, false));//read minutes without update (false)
  Serial.print(":");
  Serial.print(RTC.get(DS1307_SEC, false));//read seconds
  Serial.print("      ");                 // some space for a more happy life
  Serial.print(RTC.get(DS1307_DATE, false));//read date
  Serial.print("/");
  Serial.print(RTC.get(DS1307_MTH, false));//read month
  Serial.print("/");
  Serial.print(RTC.get(DS1307_YR, false)); //read year
  Serial.println();


	// Reading temperature for humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
	DATA_T data;
	data.humidity = dht.readHumidity();		  // Read humidity (percent)
	data.temp_c = dht.readTemperature(false);	   // Read temperature as Fahrenheit
	if(isnan(data.humidity) || isnan(data.temp_c)) {
		Serial.println("Failed to read from DHT sensor!");
		blink(2);
		delay(500);
	} else {

		data.timestamp = now();
		data.v = ESP.getVcc() / 1000.;
		Serial.print("TIME:");
		Serial.print(data.timestamp);
		Serial.print(", T:");
		Serial.print(data.temp_c);
		Serial.print(", H:");
		Serial.print(data.humidity);
		Serial.print(", V:");
		Serial.println(data.v);
		blink();


		DATA_T odata;
		// send first what is in storage if any
		bool ok = false;
		File storage = SPIFFS.open(STORAGE_NAME, "r");
		if(!storage) {
			Serial.println("Failed to open storage file for reading");
		} else {
			ok = true;
			while(storage.available()) {
				storage.read((uint8_t*)&odata, sizeof(odata));
				ok = send_sensor_data(odata);
				if(!ok) {
					break;
				}
			}
			storage.close();
		}
		if(ok) {
			// remove fully read file
			if(!SPIFFS.remove(STORAGE_NAME)) {
				Serial.println("Error removing storage file");
			}
		}
		ok = send_sensor_data(data);
		if(!ok) {
			// save to temporary file
			File storage = SPIFFS.open(STORAGE_NAME, "a+");
			if(!storage) {
				Serial.println("Failed to open storage file for appending");
			} else {
				storage.write((uint8_t*)&data, sizeof(data));
				storage.close();
			}
			blink(3);
			delay(500);
		}
	}
	ESP.deepSleep(SLEEP_SECS * 1000000L, WAKE_RF_DEFAULT);
//	delay(SLEEP_SECS * 1000);
}
