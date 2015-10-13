#include "DHT11DeepSleep.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "FS.h"
#include <DHT.h>
#include <Wire.h>
#include <DS1307.h>
#include <Time.h>
#include <HttpClient.h>

#include "config.h"

// To read VCC voltage
// TOUT pin has to be disconnected in this mode.
ADC_MODE(ADC_VCC);

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

// time to sleep between measurements
#define SLEEP_SECS 10

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30*1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

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
		RTC.set(DS1307_YR, year() - 2000);         //set the year
		RTC.start();
	}

	fs_ok = SPIFFS.begin();
	if(!fs_ok) {
		Serial.println("Formatting storage");
		SPIFFS.format();
		fs_ok = SPIFFS.begin();
	}
	if(!fs_ok) {
		Serial.println("Failed to start filesystem");
		blink(5);
	}

	delay(3000);

}

// return true if sent
bool send_sensor_data(DATA_T &data)
{
	bool rs = false;
	WiFiClient client;
	HttpClient http(client);

	String postStr = "T=" + String(data.temp_c) +
		"&H=" + String(data.humidity) +
		"&V=" + String(data.v) +
		"&TIME=" + String(data.timestamp);

	http.beginRequest();
	int err = http.post(master_server, "/dht11");
	if (err == 0)
	{
		http.sendHeader("Content-Type", "application/x-www-form-urlencoded");
		http.sendHeader("Content-Length", postStr.length());
		http.println(postStr);
		http.endRequest();
		err = http.responseStatusCode();
		if (err >= 0)
		{
			Serial.print("Got status code: ");
			Serial.println(err);

			rs = err < 300;

			// Usually you'd check that the response code is 200 or a
			// similar "success" code (200-299) before carrying on,
			// but we'll print out whatever response we get

			err = http.skipResponseHeaders();
			if (err >= 0)
			{
				int bodyLen = http.contentLength();
//				Serial.print("Content length is: ");
//				Serial.println(bodyLen);
//				Serial.println();
				Serial.println("/dht11 response body:");

				// Now we've got to the body, so we can print it out
				unsigned long timeoutStart = millis();
				char body[50];
				int bptr = 0;
				// Whilst we haven't timed out & haven't reached the end of the body
				while((http.connected() || http.available()) && ((millis() - timeoutStart) < kNetworkTimeout))
				{
					if (http.available())
					{
						body[bptr++] = http.read();
						if(bptr >= sizeof(body)) {
							break;
						}
						// We read something, reset the timeout counter
						timeoutStart = millis();
					}
					else
					{
						// We haven't got any data, so let's pause to allow some to
						// arrive
						delay(kNetworkDelay);
					}
				}
				body[bptr] = 0;
				Serial.println(body);
			}
			else
			{
				Serial.print("Failed to skip response headers: ");
				Serial.println(err);
			}
		}
		else
		{
			Serial.print("Getting response failed: ");
			Serial.println(err);
		}
	} else {
		Serial.print("Connect failed: ");
		Serial.println(err);
	}
	http.stop();
	return rs;
}

time_t request_time()
{
	time_t rs = 0;
	WiFiClient client;
	HttpClient http(client);

	int err = http.get(master_server, "/epoch");
	if (err == 0)
	{
		err = http.responseStatusCode();
		if (err >= 0)
		{
//			Serial.print("Got status code: ");
//			Serial.println(err);

			// Usually you'd check that the response code is 200 or a
			// similar "success" code (200-299) before carrying on,
			// but we'll print out whatever response we get

			err = http.skipResponseHeaders();
			if (err >= 0)
			{
				int bodyLen = http.contentLength();
//				Serial.print("Content length is: ");
//				Serial.println(bodyLen);
//				Serial.println();
//				Serial.println("/epoch response body:");

				// Now we've got to the body, so we can print it out
				unsigned long timeoutStart = millis();
				char body[20];
				int bptr = 0;
				// Whilst we haven't timed out & haven't reached the end of the body
				while((http.connected() || http.available()) && ((millis() - timeoutStart) < kNetworkTimeout))
				{
					if (http.available())
					{
						body[bptr++] = http.read();
						if(bptr >= sizeof(body)) {
							break;
						}
						// We read something, reset the timeout counter
						timeoutStart = millis();
					}
					else
					{
						// We haven't got any data, so let's pause to allow some to
						// arrive
						delay(kNetworkDelay);
					}
				}
				body[bptr] = 0;
//				Serial.println(body);
				rs = String(body).toInt();
			}
			else
			{
				Serial.print("Failed to skip response headers: ");
				Serial.println(err);
			}
		}
		else
		{
			Serial.print("Getting response failed: ");
			Serial.println(err);
		}
	} else {
		Serial.print("Connect failed: ");
		Serial.println(err);
	}
	http.stop();
	return rs;
}

void loop()
{


  Serial.print(RTC.get(DS1307_HR, false)); //read the hour and also update all the values by pushing in true
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
		bool sent_all = false, ok;
		// file structure:
		// record size = sizeof(odata)
		File storage = SPIFFS.open(STORAGE_NAME, "a");
		if(!storage) {
			Serial.println("Failed to open storage file for reading/writing");
			SPIFFS.format();
			SPIFFS.begin();
			// just send current data
			send_sensor_data(data);
		} else {
			int pos;
			sent_all = true;
			storage.seek(0, SeekSet);
			Serial.print("Storage size:");
			Serial.println(storage.size());
			if(storage.size() > 0) {
				int magic = storage.read();
				Serial.print("Magic=");
				Serial.println(magic, 16);
				if(magic == DATA_T_MAGIC && ((storage.size() - 1) % sizeof(DATA_T)) == 0) {
					// read file and send unsent data
					while(storage.available()) {
						pos = storage.position();
						storage.read((uint8_t*)&odata, sizeof(odata));
						if(!odata.sent) {
							ok = send_sensor_data(odata);
							if(ok) {
								odata.sent = true;
								storage.seek(pos, SeekSet);
								storage.write((uint8_t*)&odata, sizeof(data));
							} else {
								sent_all = false;
							}
						}
					}
				}
			}
			if(sent_all) {
				storage.close();
				storage = SPIFFS.open(STORAGE_NAME, "w");
				if(!storage) {
					Serial.println("Failed to open storage file for rewriting");
					if(!SPIFFS.remove(STORAGE_NAME)) {
						Serial.println("Error removing storage file");
					}
					storage = SPIFFS.open(STORAGE_NAME, "w");
					if(!storage) {
						Serial.println("Still failed to open storage file for rewriting");
					}
				} else {
					storage.write(DATA_T_MAGIC);
					Serial.print("New storage size:");
					Serial.println(storage.size());
					pos = storage.position();
				}
			}
			ok = send_sensor_data(data);
			if(!ok) {
				// save to temporary file
				storage.write((uint8_t*)&data, sizeof(data));
			}
			Serial.print("File size:");
			Serial.println(storage.size());
			Serial.print("Data count:");
			Serial.println((storage.size()-1.) / sizeof(data));
			storage.close();
			blink(3);
			delay(500);
		}
	}
	ESP.deepSleep(SLEEP_SECS * 1000000L, WAKE_RF_DEFAULT);
//	delay(SLEEP_SECS * 1000);
}
