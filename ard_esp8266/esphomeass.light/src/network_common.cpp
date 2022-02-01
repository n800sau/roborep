#include "network_common.h"
#include <time.h>
#include <ESP8266WiFi.h>

#include "local_config.h"

NetworkCommon::NetworkCommon()
{
	strncpy(wifi_ssid1, WIFI_SSID, sizeof(wifi_ssid1));
	strncpy(wifi_password1, WIFI_PASSWORD, sizeof(wifi_password1));
	strncpy(wifi_ssid2, WIFI_SSID1, sizeof(wifi_ssid2));
	strncpy(wifi_password2, WIFI_PASSWORD1, sizeof(wifi_password2));
}

bool NetworkCommon::getLocalTime(struct tm * info, uint32_t ms) {
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

void NetworkCommon::readConfig()
{
}

bool NetworkCommon::wifi_connect(bool force_reconnect)
{
	unsigned long start = millis();

	if(!force_reconnect) {
		Serial.println("Trying to resume WiFi connection...");

		// May be necessary after deepSleep. Otherwise you may get "error: pll_cal exceeds 2ms!!!" when trying to connect
		delay(1);
		ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));

	}

	if(force_reconnect || !WiFi.resumeFromShutdown(state) || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {

		Serial.println("Reading config");
		readConfig();
		Serial.println("Reading config finished");

		if(!force_reconnect) {
			Serial.println("Cannot resume WiFi connection");
		}
		Serial.printf("Connecting to %s or %s...\n", wifi_ssid1, wifi_ssid2);
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
			return false;
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
	return true;
}

void NetworkCommon::sync_time()
{
	unsigned long start = millis();
	configTime(TIMEZONE, "pool.ntp.org", "time.nist.gov", "time.windows.com");
	struct tm tmstruct;
	delay(2000);
	tmstruct.tm_year = 0;
	getLocalTime(&tmstruct, 5000);
	Serial.printf("\nNow is : %d-%02d-%02d %02d:%02d:%02d\n",
		(tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
	unsigned long duration = millis() - start;
	Serial.printf("NTP Duration: %f\n", duration * 0.001);
}

void NetworkCommon::wifi_shutdown()
{
	WiFi.shutdown(state);
	ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));
}
