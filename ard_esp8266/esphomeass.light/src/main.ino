
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
#include <LittleFS.h>

#define SD_CS_PIN 4

#include "local_config.h"

const char thingName[] = "homeass_light";

const char flash_fname[] = "/config.json";
const char fname[] = "config.json";
const char fname_installed[] = "config_installed.json";

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
#define RESET_PIN 5

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HASensor moisture("moisture"); // "" is unique ID of the sensor. You should define your own ID.

#define STRING_LEN 50

char wifi_ssid1[STRING_LEN] = WIFI_SSID;
char wifi_password1[STRING_LEN] = WIFI_PASSWORD;
char wifi_ssid2[STRING_LEN] = WIFI_SSID1;
char wifi_password2[STRING_LEN] = WIFI_PASSWORD1;
char mqttServer[STRING_LEN] = "192.168.1.50";
char mqttUserName[STRING_LEN] = "user1";
char mqttUserPassword[STRING_LEN] = "password1";
float sensorLowerLevel = 0;
float sensorUpperLevel = 100;

#define JSON_READ_STRING(strvar, key) { \
		if(doc.containsKey(key)) \
			strncpy(strvar, doc[key], sizeof(strvar)); \
	}
#define JSON_READ_NUMBER(numvar, key) { \
		if(doc.containsKey(key)) \
			numvar = doc[key]; \
	}

void readConfig()
{
	if(!LittleFS.begin()) {
		Serial.println("LittleFS mount failed");
	} else {
		File file = LittleFS.open(flash_fname, "r");
		if (!file) {
			Serial.printf("Failed to open flash %s for reading\n", flash_fname);
		} else {
			StaticJsonDocument<1024> doc;
			DeserializationError error = deserializeJson(doc, file);
			if (error)
			{
				Serial.println(F("Failed to read flash configuration file, using default"));
			} else {
				JSON_READ_STRING(wifi_ssid1, "SSID1")
				JSON_READ_STRING(wifi_password1, "PASSWORD1")
				JSON_READ_STRING(wifi_ssid2, "SSID2")
				JSON_READ_STRING(wifi_password2, "PASSWORD2")
				JSON_READ_STRING(mqttServer, "MQTTSERVER")
				JSON_READ_STRING(mqttUserName, "MQTTUSER")
				JSON_READ_STRING(mqttUserPassword, "MQTTPASSWORD")
				JSON_READ_NUMBER(sensorLowerLevel, "SENSORLOWERLEVEL")
				JSON_READ_NUMBER(sensorUpperLevel, "SENSORUPPERLEVEL")
			}
			file.close();
		}
	}
}

bool readUpdate()
{
	bool rs = false;
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
				Serial.printf("SD file %s exists.\n", fname);
				File file = SDFS.open(fname, "r");
				StaticJsonDocument<1024> doc;
				DeserializationError error = deserializeJson(doc, file);
				file.close();
				if (error)
				{
					Serial.printf("Failed to read SD file %s\n", fname);
				} else {
					file = LittleFS.open(flash_fname, "w");
					// Serialize JSON to file
					if(serializeJson(doc, file) == 0) {
						Serial.printf("Failed to write to flash file %s\n", flash_fname);
					} else {
						rs = true;
						File cfile = SDFS.open(fname_installed, "w");
						// Serialize JSON to file
						if(serializeJson(doc, cfile) == 0) {
							Serial.printf("Failed to write to SD file %s\n", fname_installed);
						}
						cfile.close();
					}
					file.close();
				}
			} else {
				Serial.printf("SD file %s doesn't exist.\n", fname);
			}
		} else {
			Serial.println("SD not mounted");
		}
	}
	return rs;
}

void setup() {

	Serial.begin(74880);
//	Serial.setDebugOutput(true);	// If you need debug output

	bool force_reconnect = false;

	unsigned long start = millis();

	if(digitalRead(RESET_PIN) == LOW) {
		Serial.println("Read update");
		force_reconnect = readUpdate();
	}

	readConfig();
	// ---
	// Here you can do whatever you need to do that doesn't need a WiFi connection.
	// ---

	// Unique ID must be set!
	byte mac[WL_MAC_ADDR_LENGTH+3];
	WiFi.macAddress(mac);
	memcpy(mac+WL_MAC_ADDR_LENGTH, "000", 3);
	device.setUniqueId(mac, sizeof(mac));

	// set device's details (optional)
	device.setName(thingName);
	device.setSoftwareVersion("1.0.1");

	// configure sensor (optional)
	moisture.setUnitOfMeasurement("%");
	moisture.setDeviceClass("moisture");
	moisture.setIcon("mdi:home");
	moisture.setName("Dirt moisture");

	mqtt.begin(mqttServer, mqttUserName, mqttUserPassword);

	{

		if(!force_reconnect) {
			Serial.println("Trying to resume WiFi connection...");

			// May be necessary after deepSleep. Otherwise you may get "error: pll_cal exceeds 2ms!!!" when trying to connect
			delay(1);
			ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));

		}

		if(force_reconnect || !WiFi.resumeFromShutdown(state) || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {


			Serial.println(force_reconnect ? "Reconnecting ..." : "Cannot resume WiFi connection, connecting via begin...");
			WiFi.persistent(false);

			WiFi.mode(WIFI_STA);
			// Register multi WiFi networks
			wifiMulti.addAP(wifi_ssid1, wifi_password1);
			wifiMulti.addAP(wifi_ssid2, wifi_password2);

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


	moisture.setValue(analogRead(A0));
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
