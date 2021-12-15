#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.
#include <IotWebConfMultipleWifi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "local_config.h"


// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "homeassThing";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "proniknovenie";

#define CONFIG_FILE_NAME "config.json"

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "mqt2"

void handleRoot();
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

DNSServer dnsServer;
WebServer server(80);

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);

iotwebconf::ChainedWifiParameterGroup chainedWifiParameterGroups[] = {
	iotwebconf::ChainedWifiParameterGroup("wifi1"),
	iotwebconf::ChainedWifiParameterGroup("wifi2")
};

iotwebconf::MultipleWifiAddition multipleWifiAddition(
	&iotWebConf,
	chainedWifiParameterGroups,
	sizeof(chainedWifiParameterGroups)	/ sizeof(chainedWifiParameterGroups[0])
);

iotwebconf::OptionalGroupHtmlFormatProvider optionalGroupHtmlFormatProvider;

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
//#define CONFIG_PIN D2
#define CONFIG_PIN 4

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
//#define STATUS_PIN LED_BUILTIN

#define STRING_LEN 128

char mqttServerValue[STRING_LEN]="192.168.1.50";
char mqttUserNameValue[STRING_LEN]="";
char mqttUserPasswordValue[STRING_LEN]="";

// -- You can also use namespace formats e.g.: iotwebconf::ParameterGroup
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);

#define NUMSTRING_LEN 6
char sensorMinValue[NUMSTRING_LEN]="0";
char sensorMaxValue[NUMSTRING_LEN]="100";

IotWebConfParameterGroup sensorGroup = IotWebConfParameterGroup("sensor", "Sensor configuration");
IotWebConfNumberParameter sensorMinParam = IotWebConfNumberParameter("0% Value", "sensorMin", sensorMinValue, NUMSTRING_LEN);
IotWebConfNumberParameter sensorMaxParam = IotWebConfNumberParameter("100% Value", "sensorMax", sensorMaxValue, NUMSTRING_LEN);

bool needReset = false;

#define LED_PIN LED_BUILTIN

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HASwitch led("led", false); // "led" is unique ID of the switch. You should define your own ID.
HASensor gas("gas"); // "gas" is unique ID of the sensor. You should define your own ID.

// RCWL-0516
#define MRM_PIN 5
HABinarySensor mrm("RCWL-0516", "motion", false);

void onBeforeSwitchStateChanged(bool state, HASwitch* s)
{
		// this callback will be called before publishing new state to HA
		// in some cases there may be delay before onStateChanged is called due to network latency
}

void onSwitchStateChanged(bool state, HASwitch* s)
{
	Serial.print("LED set ");
	Serial.println(state);
	digitalWrite(LED_PIN, (state ? HIGH : LOW));
}

void readConfigFile()
{
	LittleFS.begin();
	File configFile = LittleFS.open(CONFIG_FILE_NAME, "r");
	if (configFile)
	{
		Serial.println(F("Reading config file"));
		StaticJsonDocument<1024> doc;

		DeserializationError error = deserializeJson(doc, configFile);
		configFile.close();

		if (error)
		{
			Serial.println(F("Failed to read file, using default configuration"));
			return;
		}
		JsonObject documentRoot = doc.as<JsonObject>();

		// -- Apply JSON configuration.
		iotWebConf.getRootParameterGroup()->loadFromJson(documentRoot);
		iotWebConf.saveConfig();

		// -- Remove file after finished loading it.
		LittleFS.remove(CONFIG_FILE_NAME);
	}
	else
	{
		Serial.println(F("Config file not found, skipping."));
	}
	
	LittleFS.end();
}


void setup() {
	Serial.begin(115200);
	Serial.println("Starting...");

	// Unique ID must be set!
	byte mac[WL_MAC_ADDR_LENGTH];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));

//	pinMode(LED_PIN, OUTPUT);
//	digitalWrite(LED_PIN, LOW);

	mqttGroup.addItem(&mqttServerParam);
	mqttGroup.addItem(&mqttUserNameParam);
	mqttGroup.addItem(&mqttUserPasswordParam);

	sensorGroup.addItem(&sensorMinParam);
	sensorGroup.addItem(&sensorMaxParam);

//	iotWebConf.setStatusPin(STATUS_PIN);
	iotWebConf.setConfigPin(CONFIG_PIN);
	iotWebConf.addParameterGroup(&mqttGroup);
	iotWebConf.addParameterGroup(&sensorGroup);
	iotWebConf.setConfigSavedCallback(&configSaved);
	iotWebConf.setFormValidator(&formValidator);
	iotWebConf.setWifiConnectionTimeoutMs(10000);

	multipleWifiAddition.init();

	// -- Initializing the configuration.
	bool validConfig = iotWebConf.init();
	if(!validConfig) {
		Serial.println("Config invalid");
	}

	readConfigFile();

	// -- Set up required URL handlers on the web server.
	server.on("/", handleRoot);
	server.on("/config", []{ iotWebConf.handleConfig(); });
	server.onNotFound([](){ iotWebConf.handleNotFound(); });

	// set device's details (optional)
	device.setName(thingName);
	device.setSoftwareVersion("1.0.0");

	// handle switch state
	led.onBeforeStateChanged(onBeforeSwitchStateChanged); // optional
	led.onStateChanged(onSwitchStateChanged);
	led.setName("ESP LED"); // optional

	// configure sensor (optional)
	gas.setUnitOfMeasurement("µg/m³");
	gas.setDeviceClass("pm1");
	gas.setIcon("mdi:home");
	gas.setName("Home pollution");

	mrm.setName("MRM sensor");

	mqtt.begin(mqttServerValue, mqttUserNameValue, mqttUserPasswordValue);

//	IPAddress broker_addr;
//	if(broker_addr.fromString(mqttServerValue)) {
//		mqtt.begin(broker_addr, mqttUserNameValue, mqttUserPasswordValue);
//	} else {
//		Serial.print(F("Can not parse address "));
//		Serial.println(mqttServerValue);
//	}
//	mqtt.begin(IPAddress(192,168,1,50));

}

unsigned long lastSentAt = millis();
double lastValue = 0;
bool lastMRM = false;

void loop() {
	// -- doLoop should be called as frequently as possible.
	iotWebConf.doLoop();
	mqtt.loop();
	if ((millis() - lastSentAt) >= 5000) {
		lastSentAt = millis();
		lastValue = analogRead(A0);
		gas.setValue(lastValue);
		lastMRM = digitalRead(MRM_PIN);
		Serial.println(lastMRM ? "Found": "Clear");
		mrm.setState(lastMRM);

		// Supported data types:
		// uint32_t (uint16_t, uint8_t)
		// int32_t (int16_t, int8_t)
		// double
		// float
		// const char*
	}
	if (needReset)
	{
		Serial.println("Rebooting after 1 second.");
		iotWebConf.delay(1000);
		ESP.restart();
	}
}

/**
 * Handle web requests to "/" path.
 */
void handleRoot()
{
	String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
	s += "<title>IotWebConf 06 MQTT App</title></head><body>MQTT App demo";
	s += "<ul>";
	s += "<li>MQTT server: ";
	s += mqttServerValue;
	s += "</ul>";
	s += "Go to <a href='config'>configure page</a> to change values.";
	s += "<br>Current val is " + String(lastValue);
	s += "<br>Current mrm is " + String(lastMRM);
	s += "</body></html>\n";

	server.send(200, "text/html", s);
}

void configSaved()
{
	Serial.println("Configuration was updated.");
	needReset = true;
}

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
	Serial.println("Validating form.");
	// -- Note: multipleWifiAddition.formValidator() should be called, as
	// we have override this setup.
	bool valid = multipleWifiAddition.formValidator(webRequestWrapper);

	return valid;
}
