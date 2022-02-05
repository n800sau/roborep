#include "ahnetwork_common.h"

AHNetworkCommon::AHNetworkCommon(const char *thingName):
	NetworkCommon(), sensorLowerLevel(0), sensorUpperLevel(100), client(), device(), mqtt(client, device)
{
	strncpy(this->thingName, thingName, sizeof(this->thingName));
	strncpy(mqttServer, "192.168.1.50", sizeof(mqttServer));
	strncpy(mqttUserName, "user1", sizeof(mqttUserName));
	strncpy(mqttUserPassword, "password1", sizeof(mqttUserPassword));
}

void AHNetworkCommon::begin(int reset_pin, int cs_sd_pin)
{
	NetworkCommon::begin();
	this->reset_pin = reset_pin;
	this->cs_sd_pin = cs_sd_pin;
	if(this->reset_pin >= 0) {
		pinMode(reset_pin, INPUT_PULLUP);
	}
	// Unique ID must be set!
	byte mac[WL_MAC_ADDR_LENGTH+3];
	WiFi.macAddress(mac);
	memcpy(mac+WL_MAC_ADDR_LENGTH, "000", 3);
	device.setUniqueId(mac, sizeof(mac));
	// set device's details (optional)
	device.setName(thingName);
	device.setSoftwareVersion("1.0.1");

}

bool AHNetworkCommon::wifi_connect(bool force_reconnect)
{
	if(!LittleFS.begin()) {
		Serial.println("LittleFS mount failed");
	}

	if(digitalRead(reset_pin) == LOW) {
		Serial.println("Read update");
		force_reconnect = readUpdate();
	}

	readConfig();

	mqtt.begin(mqttServer, mqttUserName, mqttUserPassword);

	bool rs = NetworkCommon::wifi_connect(force_reconnect);

	return rs;
}

void AHNetworkCommon::readConfig()
{
	NetworkCommon::readConfig();
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

bool AHNetworkCommon::readUpdate()
{
	bool rs = false;
	if(cs_sd_pin > -1) {
		SDFS.setConfig(SDFSConfig(cs_sd_pin, SPI_HALF_SPEED));
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
						if(!file) {
							Serial.printf("Failed to open file %s for writing\n", flash_fname);
						}
						// Serialize JSON to file
						if(serializeJson(doc, file) == 0) {
							Serial.printf("Failed to write to flash file %s\n", flash_fname);
							Serial.print(serializeJson(doc, Serial));
							Serial.println();
						} else {
							rs = true;
							File cfile = SDFS.open(fname_installed, "w");
							// Serialize JSON to file
							if(serializeJson(doc, cfile) == 0) {
								Serial.printf("Failed to write to SD file %s\n", fname_installed);
							}
							cfile.close();
							Serial.println("Update finished");
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
	}
	return rs;
}
