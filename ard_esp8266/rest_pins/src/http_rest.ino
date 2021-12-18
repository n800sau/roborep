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
#include <LittleFS.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <uri/UriBraces.h>
#include <uri/UriRegex.h>


#include "local_config.h"

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *ap_password = WIFI_PASSWORD;

ESP8266WebServer server(80);

#define MAX_NAME_SIZE 20
char www_username[MAX_NAME_SIZE] = "";
char www_password[MAX_NAME_SIZE] = "";
// allows you to set the realm of authentication Default:"Login Required"
const char* www_realm = "Rest Pins Login Required";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";

#define FS LittleFS

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

bool is_authenticated()
{
	bool rs = (www_username[0] && www_password[0]) ? server.authenticate(www_username, www_password) : true;
	if (!rs)
		//Basic Auth Method with Custom realm and Failure Response
		//return server.requestAuthentication(BASIC_AUTH, www_realm, authFailResponse);
		//Digest Auth Method with realm="Login Required" and empty Failure Response
		//return server.requestAuthentication(DIGEST_AUTH);
		//Digest Auth Method with Custom realm and empty Failure Response
		//return server.requestAuthentication(DIGEST_AUTH, www_realm);
		//Digest Auth Method with Custom realm and Failure Response
	{
		server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
	}
	return rs;
}

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback()
{
	Serial.println("Should save config");
	shouldSaveConfig = true;
}

void setup()
{
	Serial.begin(115200);

	bool force_manager = false;

	if(FS.begin()) {
		Serial.println("mounted file system");
		if (FS.exists("/config.json")) {
			//file exists, reading and loading
			Serial.println("reading config file");
			File configFile = FS.open("/config.json", "r");
			if (configFile) {
				Serial.println("opened config file");
				size_t size = configFile.size();
				// Allocate a buffer to store contents of the file.
				std::unique_ptr<char[]> buf(new char[size]);
				configFile.readBytes(buf.get(), size);
				DynamicJsonDocument json(1024);
				auto deserializeError = deserializeJson(json, buf.get());
				serializeJson(json, Serial);
				if(!deserializeError)
				{
					Serial.println("\nparsed json");
					strcpy(www_username, json["username"]);
					strcpy(www_password, json["password"]);
				} else {
					Serial.println("failed to load json config");
				}
				configFile.close();
			}
		} else {
			force_manager = true;
		}
	} else {
		Serial.println("failed to mount FS");
	}

	WiFiManagerParameter username_param("username", "Username", www_username, MAX_NAME_SIZE);
	WiFiManagerParameter password_param("password", "Password", www_password, MAX_NAME_SIZE);

	//WiFiManager
	//Local intialization. Once its business is done, there is no need to keep it around
	WiFiManager wifiManager;

	wifiManager.setSaveConfigCallback(saveConfigCallback);

	wifiManager.setTimeout(180);

	wifiManager.addParameter(&username_param);
	wifiManager.addParameter(&password_param);

	//reset settings - for testing
//	wifiManager.resetSettings();

	String APname = String("RestPins_") + String(system_get_chip_id(), HEX);
	Serial.print("APname:");
	Serial.println(APname);

	if(!(force_manager ? wifiManager.startConfigPortal(APname.c_str()) : wifiManager.autoConnect(APname.c_str(), ap_password))) {
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

	//read updated parameters
	strcpy(www_username, username_param.getValue());
	strcpy(www_password, password_param.getValue());
	Serial.println("The values in the file are: ");
	Serial.println("\tusername : " + String(www_username));
	Serial.println("\tpassword : " + String(www_password));

	//save the custom parameters to FS
	if(shouldSaveConfig) {
		Serial.println("saving config");
		DynamicJsonDocument json(1024);
		json["username"] = www_username;
		json["password"] = www_password;

		File configFile = FS.open("/config.json", "w");
		if(!configFile) {
			Serial.println("failed to open config file for writing");
		} else {
			serializeJson(json, Serial);
			serializeJson(json, configFile);
			configFile.close();
		}
	}

	server.on("/", []() {
		if(is_authenticated()) {
			server.send(200, "text/plain", "Login OK");
		}
	});

// OUTPUT, OUTPUT_OPEN_DRAIN, INPUT, INPUT_PULLUP
	server.on(UriRegex("^\\/mode\\/([iuod])\\/([0-9]+)$"), []() {
		if(is_authenticated()) {
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
			} else {
				server.send(404, "text/plain", "pin: '" + s_pin + "' not found");
			}
		}
	});

	server.on(UriRegex("^\\/(dwrite|writed|dw|wd)\\/([0-9]+)\\/([01])$"), []() {
		if(is_authenticated()) {
			String s_pin = server.pathArg(1);
			int pin = find_pin(s_pin);
			int val = server.pathArg(2).toInt();
			if(pin >= 0) {
				digitalWrite(pin, val ? HIGH : LOW);
				server.send(200, "text/plain", "{\"pin\":\"" + s_pin + "\",\"pin_index\":" + pin + ",\"val\":" + val + "}\n");
			} else {
				server.send(404, "text/plain", "pin: '" + s_pin + "' not found");
			}
		}
	});

	server.on(UriRegex("^\\/(aw|wa|awrite|writea)\\/([0-9]+)\\/([0-9]+)$"), []() {
		if(is_authenticated()) {
			String s_pin = server.pathArg(1);
			int pin = find_pin(s_pin);
			if(pin >= 0) {
				int val = server.pathArg(2).toInt();
				analogWrite(pin, val);
				server.send(200, "text/plain", "{\"pin\":\"" + s_pin + "\",\"pin_index\":" + pin + ",\"val\":" + val + "}\n");
			} else {
				server.send(404, "text/plain", "pin: '" + s_pin + "' not found");
			}
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
