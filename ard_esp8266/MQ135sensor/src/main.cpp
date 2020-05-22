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
#include <AM2320.h>

#include "config.h"

#define SETTINGS_INIT_SUM 0x1534

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define HOSTNAME "mq135"

#define LED_PIN 15
Blinker blinker;

Ticker send_ticker;

#define VOLTAGE_RESOLUTION (1./(10./47.))
#define RL_MODULE 10.
#define RL RL_MODULE*(47+10)/(RL_MODULE+47+10)

/***********************Software Related Macros************************************/
#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
String sensor_id = "MQ135";
MQUnifiedsensor mq135("ESP8266", VOLTAGE_RESOLUTION, 10, A0, "MQ-135");

// Create an instance of sensor
AM2320 th_sensor;

#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm

// Parameters to model temperature and humidity dependence
#define CORA = 0.00035
#define CORB = 0.02718
#define CORC = 1.39538
#define CORD = 0.0018
#define CORE = -0.003333333
#define CORF = -0.001923077
#define CORG = 1.130128205

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
	float co2;
	float h;
	float t;
} val_t;

val_t cur_data = {.ts=0, .co2=0, .h=0, .t=0};

float sensorMinValue = -1, sensorMaxValue = -1;

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

/***************************** MQCalibration ****************************************
Remarks: This function assumes that the sensor is in clean air. It use
	       MQResistance to calculates the sensor resistance in clean air
	       and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
	       10, which differs slightly between different sensors.
************************************************************************************/
class MQCalibration {

		int step;
		float r0;
		Ticker t;
	public:

		bool start()
		{
			bool rs = is_running();
			if(rs) {
				Serial.println("Calibration already running");
			} else {
				Serial.printf("Calibration started (it will takes %d secs)\n", (CALIBRATION_SAMPLE_TIMES*CALIBRATION_SAMPLE_INTERVAL)/1000);
				r0 = 0;
				step = 0;
				t.attach_ms(CALIBRATION_SAMPLE_INTERVAL, std::bind(&MQCalibration::do_step, this));
			}
			return !rs;
		}

		void do_step()
		{
			mq135.update();
			r0 += mq135.calibrate(RatioMQ135CleanAir);

			if((++step) >= CALIBRATION_SAMPLE_TIMES)
			{
				stop();
			}
		}

		void stop()
		{
			t.detach();
			settings.r0 = r0/CALIBRATION_SAMPLE_TIMES;              //calculate the average value
			if(isinf(r0)) {
				Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply");
			} else if(r0 == 0) {
				Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
			} else {
				save_settings();
				Serial.printf("Calibration finished (r0:%.2f)", settings.r0);
			}
		}

		bool is_running()
		{
			return t.active();
		}

		int percent()
		{
			return (step * 100)/CALIBRATION_SAMPLE_TIMES;
		}
};


//MQCalibration calibr;

// 25 sec
void run_calibration()
{

	Serial.print("Calibrating please wait.");
	float calcR0 = 0;
	for(int i=0; i<10; i++)
	{
		mq135.update(); // Update data, the arduino will be read the voltage on the analog pin
		calcR0 += mq135.calibrate(RatioMQ135CleanAir);
		Serial.print(".");
	}
	Serial.println("  done!.");
	settings.r0 = calcR0 / 10.;
	Serial.print("R0=");
	Serial.println(settings.r0);
	save_settings();

//	calibr.start();
}

void printVals(Print &p=Serial, String sep=" ")
{
	if(settings.r0) {
		p.print("co2:");
		p.print(cur_data.co2);
		p.print( "ppm" );
		p.print(sep);
		p.print("h:");
		p.print(cur_data.h);
		p.print(sep);
		p.print("t:");
		p.print(cur_data.t);
		p.print(sep);
	}
	p.println();
//	if(calibr.is_running()) {
//		p.printf("Calibrating...%d%%", calibr.percent());
//		p.print(sep);
//		p.println();
//	}
}

const String home_link = "<a href=\"/\">Home</a>";

String wrap_html(String body, String head="")
{
	return "<html><head>"
		"<style>"
		"body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }"
		"</style>"
		"<title>" HOSTNAME "</title>" + head + "</head><body>" + body + "</body></html";
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

void send_data()
{
	if(wifiConnected && udpConnected && settings.r0) {
		Serial.print("t=");
		Serial.println(cur_data.t);
		Serial.print("h=");
		Serial.println(cur_data.h);
		Serial.print("co2=");
		Serial.println(cur_data.co2);
		String js = "{\"sensor_id\":\"" + sensor_id + "\",\"ts\":" + String(cur_data.ts) +
			",\"temperature\":" + String(cur_data.t) + ",\"humidity\":" + String(cur_data.h);
		js += ",\"co2\":" + String(cur_data.co2);
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
	page.printf("co2: %.2f (range: %.2f..%.2f)<br/>%d-%02d-%02d %02d:%02d:%02d UTC<br>ts: %li<br/>", cur_data.co2, sensorMinValue, sensorMaxValue,
		(tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec, cur_data.ts);
	printVals(page, "<br/>");
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

void th_update()
{
	// sensor.measure() returns boolean value
	// - true indicates measurement is completed and success
	// - false indicates that either sensor is not ready or crc validation failed
	//	 use getErrorCode() to check for cause of error.
	if (th_sensor.measure()) {
		cur_data.t = th_sensor.getTemperature();
		cur_data.h = th_sensor.getHumidity();
	} else {
		int errorCode = th_sensor.getErrorCode();
		switch (errorCode) {
			case 1:
				Serial.println("ERR: Sensor is offline");
				break;
			case 2:
				Serial.println("ERR: CRC validation failed.");
				break;
		}
	}
}

void sensor_update()
{
//	if(!calibr.is_running()) {
		th_update();
		mq135.setR0(settings.r0);
		mq135.update();
		cur_data.ts = time(NULL);
		mq135.setA(110.47); mq135.setB(-2.862); // Configurate the ecuation values to get CO2 concentration 
		cur_data.co2 = mq135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
//	}
}

void setup()
{
	blinker.begin(LED_PIN);
	digitalWrite(LED_PIN, HIGH);
	Serial.begin(115200);
	Serial.println();
	WiFi.mode(WIFI_STA);
	load_settings();

	mq135.init();
	mq135.setRegressionMethod(1);
	Serial.print("RL=");
	Serial.print(RL);
	mq135.setRL(RL);

	th_sensor.begin();

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
			digitalWrite(LED_PIN, LOW);
			Serial.println("Config time");
			configTime(0, 0, "pool.ntp.org", "time.nist.gov");
			if(MDNS.begin(HOSTNAME)) {
				Serial.println("MDNS responder " HOSTNAME " started");
				// Add service to MDNS-SD
				MDNS.addService("http", "tcp", 80);
			}
		}
	}
	sensor_update();
	delay(10);
}
