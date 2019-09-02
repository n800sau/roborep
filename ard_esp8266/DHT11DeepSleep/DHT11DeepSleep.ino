#include "DHT11DeepSleep.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <DHT.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <Time.h>
#include <HttpClient.h>

#include "config.h"

// To read VCC voltage
// TOUT pin has to be disconnected in this mode.
ADC_MODE(ADC_VCC);

#define DHTTYPE DHT11
#define DHTPIN	5

//const char *master_server = "192.168.1.153";
//const char *master_server = "192.168.1.9";
char master_server[20] = "sensgate";

const int s_count = std::min(20U, (4096 - sizeof(HEADER_T)) / sizeof(DATA_T));

const char* ssid	 = SSID;
const char* password = PASSWORD;

int LED_PIN = 4; // LED is attached to ESP8266 pin 4.

// time to sleep between measurements
#define SLEEP_SECS 600

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

HEADER_T header;

void setup()
{
	Serial.begin(115200);

	EEPROM.begin(s_count * sizeof(DATA_T) + sizeof(HEADER_T));
	EEPROM.get(0, header);
	if(header.magic != DATA_MAGIC) {
		// init EEPROM
		Serial.println("Init storage header");
		header.magic = DATA_MAGIC;
		header.send_first = header.send_last = -1;
		EEPROM.put(0, header);
		EEPROM.commit();
	} else {
		if(header.send_first >= s_count || header.send_last >= s_count) {
			header.send_first = header.send_last = -1;
			EEPROM.put(0, header);
			EEPROM.commit();
		}
	}

	Wire.begin(12, 14);


	// Connect to WiFi network
	WiFi.begin(ssid, password);
	WiFi.mode(WIFI_STA);
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
	Serial.print("Gateway address: ");
	Serial.println(WiFi.gatewayIP());
	strncpy(master_server, WiFi.gatewayIP().toString().c_str(), sizeof(master_server));

	dht.begin();		   // initialize temperature sensor

	pinMode(LED_PIN, OUTPUT); // Set LED pin (5) as an output
	digitalWrite(LED_PIN, LOW);

	setSyncProvider(request_time);  //set function to call when sync required

	delay(3000);

}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
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
		"&TIME=" + String(data.timestamp) +
		"&ID=" + ID;

	Serial.print("Sending:");
	Serial.println(postStr);
	http.beginRequest();
	int err = http.post(master_server, "/dht11");
	if (err == 0)
	{
		http.sendHeader("Content-Type", "application/x-www-form-urlencoded");
		http.sendHeader("Content-Length", postStr.length());
		http.println(postStr);
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
				Serial.print("/dht11 failed to skip response headers: ");
				Serial.println(err);
			}
		}
		else
		{
			Serial.print("/dht11 getting response failed: ");
			Serial.println(err);
		}
	} else {
		Serial.print("/dht11 connect failed: ");
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
				RTC.set(rs);
			}
			else
			{
				Serial.print("Failed to skip response headers: ");
				Serial.println(err);
			}
		}
		else
		{
			Serial.print("/epoch getting response failed: ");
			Serial.println(err);
		}
	} else {
		Serial.print("/epoch connect failed: ");
		Serial.println(err);
	}
	http.stop();
	return (rs == 0) ? RTC.get() : rs;
}

void digitalClockDisplay(){
	// digital clock display of the time
	Serial.print(hour());
	printDigits(minute());
	printDigits(second());
	Serial.print(" ");
	Serial.print(day());
	Serial.print(" ");
	Serial.print(month());
	Serial.print(" ");
	Serial.print(year()); 
	Serial.println(); 
}


void loop()
{
	digitalClockDisplay();

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

		data.timestamp = (timeStatus() == timeNotSet) ? 0 : now();
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
		bool ok;

			Serial.print("DATA_T size:");
			Serial.println(sizeof(DATA_T));
		while(header.send_first >= 0 && header.send_last >= 0) {
			int pos = sizeof(HEADER_T) + sizeof(DATA_T) * header.send_first;
			Serial.print("Old data address:");
			Serial.println(pos);
			EEPROM.get(pos, odata);
			if(isnan(odata.humidity) || isnan(odata.temp_c)) {
				Serial.println("Bad old data. Skip it");
				ok = true;
			} else {
				ok = send_sensor_data(odata);
				if(ok) {
					Serial.println("Old data sent successfully");
				}
			}
			if(ok) {
				if(header.send_first == header.send_last) {
					header.send_first = header.send_last = -1;
				} else {
					header.send_first++;
					if(header.send_first >= s_count) {
						header.send_first = 0;
					}
				}
			} else {
				Serial.println("Can not send old data");
				break;
			}
		}
		ok = send_sensor_data(data);
		if(ok) {
			Serial.println("New data sent successfully");
		} else {
			Serial.println("Can not send new data");
			if(data.timestamp == 0) {
				Serial.println("Time is not syncronised. No storage allowed");
			} else {
				Serial.println(data.timestamp);
				Serial.println(timeStatus());
				if(header.send_last >= 0) {
					header.send_last++;
					if(header.send_last >= s_count) {
						header.send_last = 0;
					}
					if(header.send_last == header.send_first) {
						header.send_first++;
						if(header.send_first >= s_count) {
							header.send_first = 0;
						}
					}
				} else {
					header.send_first = header.send_last = 0;
				}
				int pos = sizeof(HEADER_T) + sizeof(DATA_T) * header.send_last;
				Serial.print("Write to pos:");
				Serial.println(pos);
				EEPROM.put(pos, data);
			}
			blink(3);
		}
		EEPROM.put(0, header);
		EEPROM.commit();
	}
	ESP.deepSleep(SLEEP_SECS * 1000000L, WAKE_RF_DEFAULT);
//	delay(SLEEP_SECS * 1000);
}
