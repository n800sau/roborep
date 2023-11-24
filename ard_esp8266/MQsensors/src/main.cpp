#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <Ticker.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <time.h>
#include <blinker.h>
#include <StreamString.h>
#include <MQUnifiedsensor.h>

#include "config.h"

#define SETTINGS_INIT_SUM 0x1535

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define HOSTNAME "mq2"

#define LED_PIN 15
Blinker blinker;

Ticker send_ticker;

#define VOLTAGE_RESOLUTION (1./(10./47.))
#define RL_MODULE 10.
#define RL RL_MODULE*(47+10)/(RL_MODULE+47+10)

String sensor_id = "MQ2";
MQUnifiedsensor mq2("ESP8266", VOLTAGE_RESOLUTION, 10, A0, "MQ-2");

#define RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 8989; // local port to listen on

WiFiUDP Udp;
ESP8266WebServer server(80);

boolean wifiConnected = false;
boolean udpConnected = false;

typedef struct __attribute__((packed)) {
	int32_t data_send_period;
	float r0;
	uint16_t checksum;
} settings_t;

settings_t settings = {.data_send_period = 5000, .r0 = 10};

typedef struct {
	time_t ts;
	float co;
	float raw;
	float r0;
} val_t;

val_t cur_data = {.ts=0, .co=0, .raw=0, .r0=0};

void load_settings()
{
	// Restore from EEPROM, check the checksum matches
	settings_t s;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&s);
	EEPROM.begin(sizeof(s));
	uint16_t sum = SETTINGS_INIT_SUM;
	for (size_t i=0; i<sizeof(s); i++) {
		ptr[i] = EEPROM.read(i);
		if(i<sizeof(s)-sizeof(s.checksum)) {
			sum += ptr[i];
		}
	}
	EEPROM.end();
	if(s.checksum == sum) {
		memcpy(&settings, &s, sizeof(settings));
		Serial.print("Settings loaded:");
		Serial.println(settings.data_send_period);
	} else {
		Serial.print("Settings crc failed:");
	}
}

void save_settings()
{
	// Store in "EEPROM" to restart automatically
	settings_t s;
	memcpy(&s, &settings, sizeof(s));
	s.checksum = SETTINGS_INIT_SUM;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&s);
	for(size_t i=0; i<(sizeof(s)-sizeof(s.checksum)); i++) {
		s.checksum += ptr[i];
	}
	EEPROM.begin(sizeof(s));
	for(size_t i=0; i<sizeof(s); i++) {
		EEPROM.write(i, ptr[i]);
	}
	EEPROM.commit();
	EEPROM.end();
	Serial.print("EEPROM ");
	Serial.print(s.data_send_period);
	Serial.println(" written");
}

// 25 sec
void run_calibration()
{
	Serial.print("Calibrating please wait.");
	float calcR0 = 0;
	for(int i=0; i<10; i++)
	{
		mq2.update(); // Update data, the arduino will be read the voltage on the analog pin
		calcR0 += mq2.calibrate(RatioMQ2CleanAir);
		Serial.print(".");
	}
	Serial.println("  done!.");
	settings.r0 = calcR0 / 10.;
	Serial.print("R0=");
	Serial.println(settings.r0);
	save_settings();
}

void printVals(Print &p=Serial, String sep=" ")
{
	if(settings.r0) {
		p.print(sep);
		p.print("R0: ");
		p.print(settings.r0);
		p.print(" kΩ");
		p.print(sep);
	}
	p.print("\n");
}

const String home_link = "<a href=\"/\">Home</a>";

String wrap_html(String body, String head="")
{
	return "<html>\n<head>\n<meta charset=\"UTF-8\">\n<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
		"<style>\n"
		"body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\n"
		"</style>\n"
		"<title>" HOSTNAME "</title>\n" + head + "\n</head>\n<body>\n" + body + "\n</body>\n</html>\n";
}

const String reload_script = "<script>setTimeout(function() { location.reload(true); }, 10000)</script>";

const String postForm[] = {
		"<h1>Settings</h1><br>"
		"<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/write_settings.php\">"
			"<label>Period(ms):</label><input type=\"number\" min=\"1000\" name=\"period\" value=\"", "\"></input><br>"
			"<input type=\"submit\" name=\"submit\" value=\"submit\">Submit</input>"
			"<br/>"
			"<input type=\"submit\" name=\"submit\" value=\"calibrate\">Calibrate</input>"
		"</form>"};

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

void send_udp_string(String js)
{
	// send data message
	Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
	Udp.print(js);
	Udp.endPacket();
}

StreamString time2str(time_t ts)
{
	StreamString rs;
	struct tm tmstruct;
	localtime_r(&ts, &tmstruct);
	rs.printf("%d-%02d-%02d %02d:%02d:%02d UTC", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
	return rs;
}

void send_data()
{
	if(wifiConnected && udpConnected && settings.r0) {
		Serial.println("*****************************");
		Serial.print("co=");
		Serial.println(cur_data.co);
		String js = "{\"sensor_id\":\"" + sensor_id + "\",\"ts\":" + String(cur_data.ts) + ",\"timestamp\":\"" + time2str(cur_data.ts) + "\"";
		js += ",\"co\":" + String(cur_data.co) + ",\"raw\":" + String(cur_data.raw) + ",\"r0\":" + String(cur_data.r0);
		js += "}";
		send_udp_string(js);
		blinker.blink(3);
	}
}

void make_send_ticker()
{
	if(send_ticker.active()) {
		send_ticker.detach();
	}
	send_ticker.attach_ms(settings.data_send_period, send_data);
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
			make_send_ticker();
			message = "Success";
			String submit_type = server.arg("submit");
			Serial.printf("submit_type:%s\n", submit_type.c_str());
			if(submit_type == "calibrate") {
				run_calibration(); //Calibrating the sensor. Please make sure the sensor is in clean air
			}
		} else {
			message = "Error in form";
		}
		server.send(200, "text/html", wrap_html(home_link + "<br>" + message + "<br><a href=\"/config.html\">Config</a>"));
	}
	blinker.blink(2);
}

void handleRoot()
{
	struct tm tmstruct;
	StreamString page;
	localtime_r(&cur_data.ts, &tmstruct);
	page.printf("co: %.2f ppm\n<p>\n%s\n<br>\nts: %li\n<br>\n", cur_data.co, time2str(cur_data.ts).c_str(), cur_data.ts);
	printVals(page, "<br>\n");
	server.send(200, "text/html", wrap_html(page, reload_script));
	blinker.blink(3);
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

void sensor_update()
{
	mq2.setR0(settings.r0);
	mq2.update();
	cur_data.r0 = settings.r0;
	cur_data.raw = mq2.getVoltage();
	cur_data.ts = time(NULL);
	mq2.setA(36974); mq2.setB(-3.109);
	cur_data.co = mq2.readSensor();
//	Serial.print("Read sensor:");
//	Serial.println(cur_data.co);
}

void setup()
{
	blinker.begin(LED_PIN);
	digitalWrite(LED_PIN, HIGH);
	Serial.begin(115200);
	Serial.println();
	WiFi.mode(WIFI_STA);
	load_settings();

	mq2.init();
	mq2.setRegressionMethod(0);
	Serial.print("RL=");
	Serial.print(RL);
	mq2.setRL(RL);

	server.on("/", handleRoot);
	server.on("/index.html", handleRoot);
	server.on("/config.html", []() {
		server.send(200, "text/html", wrap_html(home_link + postForm[0] + settings.data_send_period + postForm[1]));
	});
	server.on("/write_settings.php", handleForm);
	server.onNotFound(handleNotFound);
	server.begin();
	Serial.println("HTTP server started");
	make_send_ticker();
	digitalWrite(LED_PIN, LOW);
	if(!settings.r0) {
		run_calibration();
	}
	Serial.print("Ro=");
	Serial.print(settings.r0);
	Serial.print("kohm");
	Serial.print("\n");
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
			WiFi.hostname(HOSTNAME);
			digitalWrite(LED_PIN, LOW);
			Serial.println("Config time");
			configTime(0, 0, "pool.ntp.org", "time.nist.gov");
			if(MDNS.begin(HOSTNAME)) {
				Serial.println("MDNS responder " HOSTNAME " started");
				// Add service to MDNS-SD
				MDNS.addService("http", "tcp", 80);
			} else {
				Serial.println("Error setting up MDNS responder!");
			}
		}
	}
	sensor_update();
	delay(10);
}
