#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include "config.h"

const char *ssid		 = WIFI_SSID;
const char *password = WIFI_PASSWORD;
#define HOSTNAME "mq"

#define LED_PIN LED_BUILTIN

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 8989;			// local port to listen on

WiFiUDP Udp;
ESP8266WebServer server(80);

boolean wifiConnected = false;
boolean udpConnected = false;

int data_send_period = 5000;

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
	double voltage = readVoltage();
	Serial.print("V=");
	Serial.println(voltage);
	double R0 = calc_r0();
	Serial.print("R0=");
	Serial.println(R0);

	// send no data message
	Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
	Udp.print("{ \"voltage\": " + String(voltage) + "}");
	Udp.endPacket();
}

void setup()
{
	Serial.begin(115200);
	Serial.println();
	WiFi.mode(WIFI_STA);

	server.on("/", []() {
		digitalWrite(LED_PIN, HIGH);
		server.send(200, "text/plain", "No data");
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
			delay(data_send_period);
		} else {
			udpConnected = connectUDP();
		}
	} else {
		wifiConnected = connectWifi();
		if(wifiConnected) {
			configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
			if(MDNS.begin(HOSTNAME)) {
				Serial.println("MDNS responder " HOSTNAME " started");
				// Add service to MDNS-SD
				MDNS.addService("http", "tcp", 80);
			}
		}
	}
	delay(10);
}

