/*
	HTTP Advanced Authentication example
	Created Mar 16, 2017 by Ahmed El-Sharnoby.
	This example code is in the public domain.
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>

#include <uri/UriBraces.h>
#include <uri/UriRegex.h>


#include "local_config.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

ESP8266WebServer server(80);

const char* www_username = "admin";
const char* www_password = "admin";
// allows you to set the realm of authentication Default:"Login Required"
const char* www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";

struct PIN_T {
	String name;
	int pin;
} pins[] = {
	{ "1", 1},
	{ "2", 2},
	{ "4", 4},
	{ "5", 5},
};

const int PIN_COUNT = sizeof(pins)/sizeof(pins[0]);

struct PINMODE_T {
	String name;
	int mode;
} pinmodes[] = {
	{ "o", OUTPUT},
	{ "d", OUTPUT_OPEN_DRAIN},
	{ "i", INPUT},
	{ "u", INPUT_PULLUP}
};

const int PINMODE_COUNT = sizeof(pinmodes)/sizeof(pinmodes[0]);

int find_pin(String name)
{
	int rs = -1;
	for(int i=0; i<PIN_COUNT; i++) {
		if(pins[i].name.equalsIgnoreCase(name)) {
			rs = pins[i].pin;
			break;
		}
	}
	return rs;
}

void setup()
{
	Serial.begin(115200);


		//WiFiManager
		//Local intialization. Once its business is done, there is no need to keep it around
		WiFiManager wifiManager;

	wifiManager.setTimeout(180);

	//reset settings - for testing
//	wifiManager.resetSettings();

	String APname = String("RestAutoConnectAP_") + String(system_get_chip_id(), HEX);
	Serial.print("APname:");
	Serial.println(APname);

	if(!wifiManager.autoConnect(APname.c_str())) {
		Serial.println("failed to connect and hit timeout");
		delay(3000);
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(5000);
	} 



//	WiFi.mode(WIFI_STA);
//	WiFi.begin(ssid, password);
//	if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//		Serial.println("WiFi Connect Failed! Rebooting...");
//		delay(1000);
//		ESP.restart();
//	}

	server.on("/", []() {
		if (!server.authenticate(www_username, www_password))
			//Basic Auth Method with Custom realm and Failure Response
			//return server.requestAuthentication(BASIC_AUTH, www_realm, authFailResponse);
			//Digest Auth Method with realm="Login Required" and empty Failure Response
			//return server.requestAuthentication(DIGEST_AUTH);
			//Digest Auth Method with Custom realm and empty Failure Response
			//return server.requestAuthentication(DIGEST_AUTH, www_realm);
			//Digest Auth Method with Custom realm and Failure Response
		{
			return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
		}
		server.send(200, "text/plain", "Login OK");
	});

// OUTPUT, OUTPUT_OPEN_DRAIN, INPUT, INPUT_PULLUP
	server.on(UriRegex("^\\/mode\\/([iuod])\\/([0-9]+)$"), []() {
		String mode = server.pathArg(0);
		String s_pin = server.pathArg(1);
		int pin = find_pin(s_pin);
		bool found = false;
		if(pin >= 0) {
			for(int i=0; i<PINMODE_COUNT; i++) {
				if(pinmodes[i].name.equalsIgnoreCase(mode)) {
					pinMode(pin, pinmodes[i].mode);
					found = true;
					break;
				}
			}
		}
		if(found) {
			server.send(200, "text/plain", "{\"pin\":\"" + s_pin + "\",\"pin_index\":" + pin + ",\"mode\":\"" + mode + "\"}\n");
			server.send(200, "text/plain", "mode: '" + mode + "' and pin: '" + s_pin + "'");
		} else {
			server.send(404, "text/plain", "pin: '" + s_pin + "' not found");
		}
	});

	server.on(UriRegex("^\\/(dwrite|writed|dw|wd)\\/([0-9]+)\\/([01])$"), []() {
		String s_pin = server.pathArg(1);
		int pin = find_pin(s_pin);
		int val = server.pathArg(2).toInt();
		if(pin >= 0) {
			digitalWrite(pin, val ? HIGH : LOW);
			server.send(200, "text/plain", "{\"pin\":\"" + s_pin + "\",\"pin_index\":" + pin + ",\"val\":" + val + "}\n");
		} else {
			server.send(404, "text/plain", "pin: '" + s_pin + "' not found");
		}
	});

	server.on(UriRegex("^\\/(aw|wa|awrite|writea)\\/([0-9]+)\\/([0-9]+)$"), []() {
		String s_pin = server.pathArg(1);
		int pin = find_pin(s_pin);
		if(pin >= 0) {
			int val = server.pathArg(2).toInt();
			analogWrite(pin, val);
			server.send(200, "text/plain", "{\"pin\":\"" + s_pin + "\",\"pin_index\":" + pin + ",\"val\":" + val + "}\n");
		} else {
			server.send(404, "text/plain", "pin: '" + s_pin + "' not found");
		}
	});

	server.begin();

	Serial.print("Open http://");
	Serial.print(WiFi.localIP());
	Serial.println("/ in your browser to see it working");
}

void loop() {
	server.handleClient();
}
