#ifndef AHNETWORK_COMMON_H
#define AHNETWORK_COMMON_H

#include <ArduinoHA.h>
#include <ArduinoJson.h>
#include <SDFS.h>
#include <LittleFS.h>

#include "network_common.h"

#define JSON_READ_STRING(strvar, key) { \
		if(doc.containsKey(key)) \
			strncpy(strvar, doc[key], sizeof(strvar)); \
	}
#define JSON_READ_NUMBER(numvar, key) { \
		if(doc.containsKey(key)) \
			numvar = doc[key]; \
	}


class AHNetworkCommon: public NetworkCommon {

	public:

		char thingName[STRING_LEN];
		char mqttServer[STRING_LEN];
		char mqttUserName[STRING_LEN];
		char mqttUserPassword[STRING_LEN];
		float sensorLowerLevel;
		float sensorUpperLevel;

		const char *flash_fname = "/config.json";
		const char *fname = "config.json";
		const char *fname_installed = "config_installed.json";
		int reset_pin;
		int cs_sd_pin;

		WiFiClient client;
		HADevice device;
		HAMqtt mqtt;

		AHNetworkCommon(const char *thingName);

		void begin(int reset_pin=-1, int cs_sd_pin=-1);

		bool wifi_connect(bool force_reconnect=false) override;
		void readConfig() override;
		bool readUpdate();

};



#endif //AHNETWORK_COMMON_H
