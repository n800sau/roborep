#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <time.h>

#include "config.h"

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
#define HOSTNAME "mq"

#define LED_PIN 4

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

char datebuf[60];

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

const String postForm = "<html>\
	<head>\
		<title>" HOSTNAME "</title>\
		<style>\
			body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
		</style>\
	</head>\
	<body>\
		<h1>POST plain text to /postplain/</h1><br>\
		<form method=\"post\" enctype=\"text/plain\" action=\"/postplain/\">\
			<input type=\"text\" name=\"hello\" value=\"world\"><br>\
			<input type=\"submit\" value=\"Submit\">\
		</form>\
		<h1>POST form data to /postform/</h1><br>\
		<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/postform/\">\
			<input type=\"text\" name=\"hello\" value=\"world\"><br>\
			<input type=\"submit\" value=\"Submit\">\
		</form>\
	</body>\
</html>";

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

void handlePlain() {
	digitalWrite(LED_PIN, HIGH);
	if (server.method() != HTTP_POST) {
		server.send(405, "text/plain", "Method Not Allowed");
	} else {
		server.send(200, "text/plain", "POST body was:\n" + server.arg("plain"));
	}
	digitalWrite(LED_PIN, LOW);
}

void handleForm() {
	digitalWrite(LED_PIN, HIGH);
	if (server.method() != HTTP_POST) {
		server.send(405, "text/plain", "Method Not Allowed");
	} else {
		String message = "POST form was:\n";
		for (uint8_t i = 0; i < server.args(); i++) {
			message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
		}
		server.send(200, "text/plain", message);
	}
	save_settings();
	digitalWrite(LED_PIN, LOW);
}

double readVoltage()
{
	double sensorValue = 0;
	const double div_coef = 10. / (47 + 10);
	for(int x = 0 ; x < 500 ; x++) //Start for loop 
	{
		sensorValue += analogRead(A0);
	}
	return sensorValue / 500 / 1023 / div_coef;
}

double calc_r0()
{
	double RS_air; //Define variable for sensor resistance
	double R0; //Define variable for R0
	double sensorVoltage = readVoltage();
Serial.println(sensorVoltage);
	RS_air = (5/sensorVoltage)-1; //Calculate RS in fresh air
	R0 = RS_air/4.4; //Calculate R0
	return R0;
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

void send_data()
{
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
}

void setup()
{
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	Serial.begin(115200);
	Serial.println();
	WiFi.mode(WIFI_STA);
	load_settings();

	server.on("/", []() {
		digitalWrite(LED_PIN, HIGH);
		struct tm tmstruct;
		localtime_r(&cur_data.ts, &tmstruct);
		snprintf(datebuf, sizeof(datebuf), "%g<br>%d-%02d-%02d %02d:%02d:%02d UTC<br>%li", cur_data.voltage, (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1,
			tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec, cur_data.ts);
		server.send(200, "text/plain", String(datebuf));
		digitalWrite(LED_PIN, LOW);
	});
	server.on("/config", []() {
		server.send(200, "text/plain", postForm);
	});
	server.on("/postplain/", handlePlain);
	server.on("/postform/", handleForm);
	server.onNotFound(handleNotFound);
	server.begin();
	Serial.println("HTTP server started");
	digitalWrite(LED_PIN, LOW);
}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(wifiConnected)
	{
		MDNS.update();
		server.handleClient();
		if(udpConnected)
		{
			send_data();
			delay(settings.data_send_period);
		} else {
			udpConnected = connectUDP();
		}
	} else {
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

