#ifndef NETWORK_COMMON_H
#define NETWORK_COMMON_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <include/WiFiState.h> // WiFiState structure details
#include <ESP8266WiFiMulti.h>

#define STRING_LEN 50

#ifndef RTC_USER_DATA_SLOT_WIFI_STATE
#define RTC_USER_DATA_SLOT_WIFI_STATE 33u
#endif

#define TIMEZONE "Australia/Sydney"

class NetworkCommon {

	public:

		NetworkCommon();

		char wifi_ssid1[STRING_LEN];
		char wifi_password1[STRING_LEN];
		char wifi_ssid2[STRING_LEN];
		char wifi_password2[STRING_LEN];

		WiFiState state;
		ESP8266WiFiMulti wifiMulti;

		virtual void begin() {};
		bool getLocalTime(struct tm * info, uint32_t ms);
		virtual bool wifi_connect(bool force_reconnect=false);
		virtual void readConfig();
		void sync_time();
		virtual void wifi_shutdown();

};

#endif //NETWORK_COMMON_H
