#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <Ticker.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <time.h>
#include <blinker.h>

#include "config.h"

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
#define HOSTNAME "mq"

#define LED_PIN 4
Blinker blinker;

Ticker ticker;

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 8989; // local port to listen on

WiFiUDP Udp;
ESP8266WebServer server(80);

boolean wifiConnected = false;
boolean udpConnected = false;

typedef struct {
	int32_t data_send_period;
	int16_t checksum;
} settings_t;

settings_t settings = {.data_send_period = 5000};

typedef struct {
	time_t ts;
	double voltage;
} val_t;

val_t cur_data = {0, 0};

char htmlbuf[120];

void load_settings()
{
	// Restore from EEPROM, check the checksum matches
	settings_t s;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&s);
	EEPROM.begin(sizeof(s));
	int16_t sum = 0x1234;
	for (size_t i=0; i<sizeof(s); i++) {
		ptr[i] = EEPROM.read(i);
		if(i<sizeof(s)-sizeof(s.checksum)) {
			sum += ptr[i];
		}
	}
	EEPROM.end();
	if (s.checksum == sum) {
		memcpy(&settings, &s, sizeof(settings));
	}
}

void save_settings()
{
	// Store in "EEPROM" to restart automatically
	settings_t s;
	memcpy(&s, &settings, sizeof(s));
	s.checksum = 0x1234;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&s);
	for (size_t i=0; i<sizeof(s)-sizeof(s.checksum); i++) s.checksum += ptr[i];
	EEPROM.begin(sizeof(s));
	for (size_t i=0; i<sizeof(s); i++) {
		EEPROM.write(i, ptr[i]);
	}
	EEPROM.commit();
	EEPROM.end();
}

const String rootPage[] = {"<html><head>"
	"<title>" HOSTNAME "</title>"
	"<script>setTimeout(function() { location.reload(true); }, 10000)</script>"
	"</head>"
	"<body>",
	"</body></html"};

const String postForm[] = {"<html>\
	<head>\
		<title>" HOSTNAME "</title>\
		<style>\
			body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
		</style>\
	</head>\
	<body>\
		<h1>Settings</h1><br>\
		<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/write_settings/\">\
			<label>Period(ms):</label><input type=\"number\" min=\"1000\" name=\"period\" value=\"", "\"></input><br>\
			<input type=\"submit\" value=\"Submit\"></input>\
		</form>\
	</body>\
</html>"};

void handleNotFound(){
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET)?"GET":"POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	for (uint8_t i=0; i<server.args(); i++){
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
}

double sensorMinValue = -1, sensorMaxValue = -1;

double readVoltage()
{
	double sensorValue = 0;
	sensorMinValue = -1;
	sensorMaxValue = -1;
	const double div_coef = 10. / (47 + 10);
	for(int x = 0 ; x < 500 ; x++) //Start for loop 
	{
		int v = analogRead(A0);
		sensorValue += v;
		if(sensorMinValue < 0 || sensorMinValue > v) {
			sensorMinValue = v;
		}
		if(sensorMaxValue < 0 || sensorMaxValue < v) {
			sensorMaxValue = v;
		}
	}
	sensorMinValue = sensorMinValue / 1023 / div_coef;
	sensorMaxValue = sensorMaxValue / 1023 / div_coef;
	Serial.print("min:");
	Serial.print(sensorMinValue);
	Serial.print(", max:");
	Serial.println(sensorMaxValue);
	return sensorValue / 500 / 1023 / div_coef;
}

double calc_r0()
{
	double RS_air; //Define variable for sensor resistance
	double R0; //Define variable for R0
	double sensorVoltage = readVoltage();
	RS_air = (5/sensorVoltage)-1; //Calculate RS in fresh air
	R0 = RS_air/4.4; //Calculate R0
	return R0;
}

void send_data()
{
	if(wifiConnected && udpConnected) {
		cur_data.voltage = readVoltage();
		cur_data.ts = time(NULL);
		Serial.print("V=");
		Serial.println(cur_data.voltage);
		double R0 = calc_r0();
		Serial.print("R0=");
		Serial.println(R0);

		// send no data message
		Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
		Udp.print("{\"ts\":" + String(cur_data.ts) + ",\"voltage\":" + String(cur_data.voltage) + "}");
		Udp.endPacket();
		blinker.blink(3);
	}
}

void make_ticker()
{
	ticker.detach();
	ticker.attach(settings.data_send_period, send_data);
}

void handleForm()
{
	if (server.method() != HTTP_POST) {
		server.send(405, "text/plain", "Method Not Allowed");
	} else {
		int v = server.arg("period").toInt();
		String message;
		if(v > 1000) {
			settings.data_send_period = v;
			save_settings();
			message = "Success\n";
			server.send(200, "text/plain", message);
			make_ticker();
		} else {
			message = "Error in form\n";
		}
		server.send(200, "text/plain", message);
	}
	blinker.blink(2);
}

// connect to wifi returns true if successful or false if not
boolean connectWifi()
{
	boolean state = true;
	int i = 0;
	WiFi.begin(ssid, password);
	Serial.println("");
	Serial.println("Connecting to WiFi");

	// Wait for connection
	Serial.print("Connecting");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
		if (i > 10){
			state = false;
			break;
		}
		i++;
	}
	if (state)
	{
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(ssid);
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
		Serial.println("");
		Serial.println("Connection failed.");
	}
	return state;
}


// connect to UDP returns true if successful or false if not
boolean connectUDP()
{
	boolean state = false;

	Serial.println();
	Serial.println("Starting UDP multicast");

	if(Udp.beginMulticast(WiFi.localIP(), ipMulti, portMulti) == 1) {
		Serial.println("Udp start successful");
		state = true;
	} else {
		Serial.println("Udp start failed");
	}

	return state;
}

void setup()
{
	blinker.begin(LED_PIN);
	digitalWrite(LED_PIN, HIGH);
	Serial.begin(115200);
	Serial.println();
	WiFi.mode(WIFI_STA);
	load_settings();

	server.on("/", []() {
		struct tm tmstruct;
		localtime_r(&cur_data.ts, &tmstruct);
		snprintf(htmlbuf, sizeof(htmlbuf), "V: %.2f (range: %.2f..%.2f)<br>%d-%02d-%02d %02d:%02d:%02d UTC<br>ts: %li", cur_data.voltage, sensorMinValue, sensorMaxValue,
			(tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1,
			tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec, cur_data.ts);
		server.send(200, "text/plain", rootPage[0] + htmlbuf + rootPage[1]);
		blinker.blink(3);
	});
	server.on("/config", []() {
		server.send(200, "text/plain", postForm[0] + settings.data_send_period + postForm[1]);
	});
	server.on("/write_settings/", handleForm);
	server.onNotFound(handleNotFound);
	server.begin();
	Serial.println("HTTP server started");
	make_ticker();
	digitalWrite(LED_PIN, LOW);
}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(wifiConnected)
	{
		MDNS.update();
		server.handleClient();
		if(!udpConnected) {
			udpConnected = connectUDP();
		}
	} else {
		blinker.stop();
		digitalWrite(LED_PIN, HIGH);
		wifiConnected = connectWifi();
		if(wifiConnected) {
			digitalWrite(LED_PIN, LOW);
			configTime(0, 0, "pool.ntp.org", "time.nist.gov");
			if(MDNS.begin(HOSTNAME)) {
				Serial.println("MDNS responder " HOSTNAME " started");
				// Add service to MDNS-SD
				MDNS.addService("http", "tcp", 80);
			}
		}
	}
	delay(10);
}

