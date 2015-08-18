#include "rest_service.h"
#include "uart_utils.h"
#include "espREST.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Ticker.h>
#include <PString.h>
#include <ArduinoJson.h>

extern "C" {
#include "user_interface.h"
}

const int ioResetPin = 4;

#define MAX_JSON_SIZE 1000

static String message;

static Ticker unsetter;

// Create espREST instance
espREST rest;
// Create an instance of the server
ESP8266WebServer rserver(8080);

static void ICACHE_FLASH_ATTR resetOff()
{
	digitalWrite(ioResetPin, 1);
}

static void ICACHE_FLASH_ATTR doReset()
{
	ESP.restart();
}

static FTYPE ICACHE_FLASH_ATTR ioReset(String param, String &result)
{
	digitalWrite(ioResetPin, 0);
	unsetter.once(1, resetOff);
	return FT_NONE;
}

static FTYPE ICACHE_FLASH_ATTR selSer(String param, String &result)
{
	uartSetPrimary(!param.toInt());
	result = String((uartIsPrimary()) ? "primary": "secondary");
	return FT_STRING;
}

static FTYPE ICACHE_FLASH_ATTR setBaud(String param, String &result)
{
	int baud = param.toInt();
	Serial.begin(baud);
	result = String(baud);
	return FT_INT;
}

static FTYPE ICACHE_FLASH_ATTR selfReset(String param, String &result)
{
	unsetter.once(1, doReset);
	return FT_NONE;
}

static FTYPE ICACHE_FLASH_ATTR getVcc(String param, String &result)
{
	result = String(ESP.getVcc());
	return FT_INT;
}

static FTYPE ICACHE_FLASH_ATTR getFreeHeap(String param, String &result)
{
	result = String(ESP.getFreeHeap());
	return FT_INT;
}

static FTYPE ICACHE_FLASH_ATTR executeRESTCommand(String command, String &result)
{
	FTYPE rs = FT_ERROR;
	char msgbuf[MAX_JSON_SIZE];
	PString pmsg(msgbuf, MAX_JSON_SIZE);
	DynamicJsonBuffer jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	if(command.equalsIgnoreCase("info")) {
		root["mode"] = wifi_get_opmode();
		const char* phymodes[] = {"", "B", "G", "N"};
		root["phy_mode"] = phymodes[(int) wifi_get_phy_mode()];
		root["channel"] = WiFi.Channel();
		root["ap"] = wifi_station_get_current_ap_id();
		root["connected"] = (bool)wifi_station_get_connect_status();
		root["auto_connect"] = (bool)wifi_station_get_auto_connect();
		root["ssid"] = WiFi.SSID();
		pmsg.begin();
		WiFi.gatewayIP().printTo(pmsg);
		root["gateway"] = String(msgbuf).c_str();
		pmsg.begin();
		WiFi.localIP().printTo(pmsg);
		root["ip"] = String(msgbuf).c_str();
		rs = FT_JSON;
	}
	if(rs == FT_JSON) {
		pmsg.begin();
		root.printTo(pmsg);
		result = String(msgbuf).c_str();
	}
	return rs;
}

void ICACHE_FLASH_ATTR setupRESTservice()
{
	pinMode(ioResetPin, OUTPUT);
	digitalWrite(ioResetPin, 1);

	// Give name and ID to device
	rest.set_id((char*)String(ESP.getChipId()).c_str());
	rest.set_name((char*)"esp8266");

	rest.function((char*)"ioreset", ioReset);

	rest.function((char*)"reset", selfReset);

	// Function to change serial
	rest.function((char*)"selser", selSer);
	rest.function((char*)"setbaud", setBaud);

	rest.function((char*)"vcc", getVcc);
	rest.function((char*)"free", getFreeHeap);

	// Command function
	rest.function((char*)"command", executeRESTCommand);

	rest.variable((char*)"message", &message);

	// Start the REST server
	rserver.begin();
	rest.begin(rserver);

}

void ICACHE_FLASH_ATTR handleRESTservice()
{
	rserver.handleClient();
}

