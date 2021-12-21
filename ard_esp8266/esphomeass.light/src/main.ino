
// Demonstrate the use of WiFi.shutdown() and WiFi.resumeFromShutdown()
// Released to public domain

// Current on WEMOS D1 mini (including: LDO, usbserial chip):
// ~85mA during normal operations
// ~30mA during wifi shutdown
//  ~5mA during deepsleep

#include <time.h>
#include <ESP8266WiFi.h>
#include <include/WiFiState.h> // WiFiState structure details
#include <ESP8266WiFiMulti.h>
#include <ArduinoHA.h>
#include <ArduinoJson.h>
#include <SDFS.h>

#define SD_CS_PIN 4

#include "local_config.h"

const char thingName[] = "homeass_light";

const char fname[] = "config.json";
File config_file;

#ifndef RTC_USER_DATA_SLOT_WIFI_STATE
#define RTC_USER_DATA_SLOT_WIFI_STATE 33u
#endif

//#define DEEPSLEEP_TIME (10e6 * 60ll)
#define DEEPSLEEP_TIME (10e6)

WiFiState state;

ESP8266WiFiMulti wifiMulti;

#define TIMEZONE "Australia/Sydney"

bool getLocalTime(struct tm * info, uint32_t ms) {
	uint32_t count = ms / 10;
	time_t now;

	time(&now);
	localtime_r(&now, info);

	if (info->tm_year > (2016 - 1900)) {
		return true;
	}

	while (count--) {
		delay(10);
		time(&now);
		localtime_r(&now, info);
		if (info->tm_year > (2016 - 1900)) {
			return true;
		}
	}
	return false;
}

#define LED_PIN LED_BUILTIN

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HASensor gas("gas"); // "gas" is unique ID of the sensor. You should define your own ID.

#define STRING_LEN 128

char mqttServerValue[STRING_LEN]="192.168.1.50";
char mqttUserNameValue[STRING_LEN]="user1";
char mqttUserPasswordValue[STRING_LEN]="password1";

void setup() {

	Serial.begin(74880);
//	Serial.setDebugOutput(true);	// If you need debug output

	Serial.println("Trying to resume WiFi connection...");

	// May be necessary after deepSleep. Otherwise you may get "error: pll_cal exceeds 2ms!!!" when trying to connect
	delay(1);

	SDFS.setConfig(SDFSConfig(SD_CS_PIN, SPI_HALF_SPEED));

	if(!SDFS.begin()) {
		Serial.println("SD initialization failed!");
	} else {
		FSInfo64 info;
		if(SDFS.info64(info)) {
			Serial.print("Size: ");
			Serial.println(info.totalBytes);
			Serial.print("Free: ");
			Serial.println(info.totalBytes-info.usedBytes);
			if(SDFS.exists(fname)) {
				Serial.printf("%s exists.\n", fname);
				config_file = SDFS.open(fname, "r");
				StaticJsonDocument<1024> doc;
				DeserializationError error = deserializeJson(doc, config_file);
				config_file.close();
				if (error)
				{
					Serial.println(F("Failed to read file, using default configuration"));
				} else {
					const char* j_ssid1 = doc["SSID1"];
					const char* j_password1 = doc["PASSWORD1"];
					const char* j_ssid2 = doc["SSID2"];
					const char* j_password2 = doc["PASSWORD2"];
					Serial.println(j_ssid1);
					Serial.println(j_ssid2);
				}
			} else {
				Serial.printf("%s doesn't exist.\n", fname);
			}
		} else {
			Serial.println("SD not mounted");
		}
	}

	// ---
	// Here you can do whatever you need to do that doesn't need a WiFi connection.
	// ---

	// Unique ID must be set!
	byte mac[WL_MAC_ADDR_LENGTH];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));

	// set device's details (optional)
	device.setName(thingName);
	device.setSoftwareVersion("1.0.1");

	// configure sensor (optional)
	gas.setUnitOfMeasurement("µg/m³");
	gas.setDeviceClass("pm1");
	gas.setIcon("mdi:home");
	gas.setName("Home pollution");

	mqtt.begin(mqttServerValue, mqttUserNameValue, mqttUserPasswordValue);

	{

		ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));
		unsigned long start = millis();

		if (!WiFi.resumeFromShutdown(state) || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
			Serial.println("Cannot resume WiFi connection, connecting via begin...");
			WiFi.persistent(false);

			WiFi.mode(WIFI_STA);
			// Register multi WiFi networks
			wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
			wifiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);

			if (wifiMulti.run(10000) != WL_CONNECTED) {
				Serial.println("WiFi not connected!");
				WiFi.mode(WIFI_OFF);
				Serial.println("Cannot connect!");
				Serial.flush();
				ESP.deepSleep(10e6, RF_DISABLED);
				return;
			}
		} else {
			Serial.println("Resumed");
		}

		Serial.print("WiFi connected: ");
		Serial.print(WiFi.SSID());
		Serial.print(" ");
		Serial.println(WiFi.localIP());

		unsigned long duration = millis() - start;
		Serial.printf("Connection duration: %f\n", duration * 0.001);
	}

	{
		unsigned long start = millis();
		configTime(TIMEZONE, "pool.ntp.org", "time.nist.gov", "time.windows.com");
		struct tm tmstruct ;
		delay(2000);
		tmstruct.tm_year = 0;
		getLocalTime(&tmstruct, 5000);
		Serial.printf("\nNow is : %d-%02d-%02d %02d:%02d:%02d\n",
			(tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);

		unsigned long duration = millis() - start;
		Serial.printf("NTP Duration: %f\n", duration * 0.001);
	}


	gas.setValue(analogRead(A0));
	mqtt.loop();
	yield();

	// ---
	// Here you can do whatever you need to do that needs a WiFi connection.
	// ---

	WiFi.shutdown(state);
	ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));

	// ---
	// Here you can do whatever you need to do that doesn't need a WiFi connection anymore.
	// ---

	Serial.println("Go deep sleep...");
	Serial.flush();
	ESP.deepSleep(DEEPSLEEP_TIME, RF_DISABLED);
}

void loop() {
	// Nothing to do here.
}
