#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <FS.h>

#include "config.h"
const char* ssid = SSID;
const char* password = PASSWORD;

Ticker flipper;

void flash_info()
{
	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();

	Serial.printf("Flash real id:	 %08X\n", ESP.getFlashChipId());
	Serial.printf("Flash real size: %u\n\n", realSize);

	Serial.printf("Flash ide	size: %u\n", ideSize);
	Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
	Serial.printf("Flash ide mode:	%s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

	if(ideSize != realSize) {
			Serial.println("Flash Chip configuration wrong!\n");
	} else {
			Serial.println("Flash Chip configuration ok.\n");
	}
}

void file_test()
{
	File helloFile = SPIFFS.open("/hello.txt", "r");
	if (!helloFile)
	{
		Serial.println("Failed to open hello.txt.");
	} else {
		String content = helloFile.readString();
		helloFile.close();
		content.trim();
		Serial.print("File content:");
		Serial.println(content);
	}
}

void setup() {
	Serial.begin(115200);
	flash_info();
	Serial.print("\nFree heap:");
	Serial.println(ESP.getFreeHeap());
	// Initialize file system.
//	SPIFFS.format();
	if (!SPIFFS.begin())
	{
		Serial.println("Failed to mount file system");
	}
	Serial.println("Booting");
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}

	// Port defaults to 8266
	// ArduinoOTA.setPort(8266);

	// Hostname defaults to esp8266-[ChipID]
	ArduinoOTA.setHostname("myesp8266");

	// No authentication by default
	ArduinoOTA.setPassword((const char *)"Zagruzka");

	ArduinoOTA.onStart([]() {
		Serial.println("Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();
	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	// flip the pin every 0.3s
	flipper.attach(5, file_test);
}

void loop() {
	ArduinoOTA.handle();
}
