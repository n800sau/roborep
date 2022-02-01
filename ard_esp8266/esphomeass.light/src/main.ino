#include "ahnetwork_common.h"

const char *thingName = "homeass_light";

//#define DEEPSLEEP_TIME (10e6 * 60ll)
#define DEEPSLEEP_TIME (10e6)

//#define RESET_PIN 5
//#define SD_CS_PIN 4

#define RESET_PIN D2
#define SD_CS_PIN D1


AHNetworkCommon ncommon(thingName);

HASensor moisture("moisture"); // "" is unique ID of the sensor. You should define your own ID.

void setup() {

	Serial.begin(74880);
	Serial.setDebugOutput(true);	// If you need debug output


	ncommon.begin(RESET_PIN, SD_CS_PIN);
	// ---
	// Here you can do whatever you need to do that doesn't need a WiFi connection.
	// ---

	if(!ncommon.wifi_connect()) {
		ESP.deepSleep(10e6, RF_DISABLED);
	}

	ncommon.sync_time();

	// configure sensor (optional)
	moisture.setUnitOfMeasurement("%");
	moisture.setDeviceClass("moisture");
	moisture.setIcon("mdi:home");
	moisture.setName("Dirt moisture");

	moisture.setValue(analogRead(A0));
	ncommon.mqtt.loop();
	yield();

	// ---
	// Here you can do whatever you need to do that needs a WiFi connection.
	// ---

//	ncommon.wifi_shutdown();

	// ---
	// Here you can do whatever you need to do that doesn't need a WiFi connection anymore.
	// ---

//	Serial.println("Go deep sleep...");
//	Serial.flush();
//	ESP.deepSleep(DEEPSLEEP_TIME, RF_DISABLED);
}

void loop() {
	moisture.setValue(analogRead(A0));
	ncommon.mqtt.loop();
	delay(1000);
}
