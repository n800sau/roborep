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
#define HOSTNAME "mq2"

#define LED_PIN 4
Blinker blinker;

Ticker ticker;

String sensor_id = "MQ2";

/************************Hardware Related Macros************************************/
#define         RL_VALUE_MQ2                 (47 + 10)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ2      (9.577)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase

/**********************Application Related Macros**********************************/
#define         GAS_HYDROGEN                  (0)
#define         GAS_LPG                       (1)
#define         GAS_METHANE                   (2)
#define         GAS_CARBON_MONOXIDE           (3)
#define         GAS_ALCOHOL                   (4)
#define         GAS_SMOKE                     (5)
#define         GAS_PROPANE                   (6)
#define         accuracy                      (0)   //for linearcurves
//#define         accuracy                    (1)   //for nonlinearcurves, un comment this line and comment the above line if calculations 
                                                    //are to be done using non linear curve equations
/*****************************Globals************************************************/
double           Ro = 0;                            //Ro is initialized to 10 kilo ohms

double sensorMinValue = -1, sensorMaxValue = -1;
const double boardResistance = 10 + 47;

double readVoltage()
{
	double sensorValue = 0;
	sensorMinValue = -1;
	sensorMaxValue = -1;
	const double div_coef = 10 / boardResistance;
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
//	Serial.print("min:");
//	Serial.print(sensorMinValue);
//	Serial.print(", max:");
//	Serial.println(sensorMaxValue);
	return sensorValue / 500 / 1023 / div_coef;
}


/****************** MQResistance ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
double MQResistance()
{
	return RL_VALUE_MQ2 * (5 - readVoltage());
}

/***************************** MQCalibration ****************************************
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use
	       MQResistance to calculates the sensor resistance in clean air
	       and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
	       10, which differs slightly between different sensors.
************************************************************************************/
double MQCalibration()
{
	int i;
	double RS_AIR_val=0,r0;

	for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {                     //take multiple samples
		RS_AIR_val += MQResistance();
		delay(CALIBRATION_SAMPLE_INTERVAL);
	}
	RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;              //calculate the average value

	r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ2;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro

	return r0;
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
       gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function uses different equations representing curves of each gas to
       calculate the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(double rs_ro_ratio, int gas_id)
{
	if ( accuracy == 0 ) {
		if ( gas_id == GAS_HYDROGEN ) {
			return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)));
		} else if ( gas_id == GAS_LPG ) {
			return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)));
		} else if ( gas_id == GAS_METHANE ) {
			return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)));
		} else if ( gas_id == GAS_CARBON_MONOXIDE ) {
			return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)));
		} else if ( gas_id == GAS_ALCOHOL ) {
			return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)));
		} else if ( gas_id == GAS_SMOKE ) {
			return (pow(10,((-2.331*(log10(rs_ro_ratio))) + 3.596)));
		} else if ( gas_id == GAS_PROPANE ) {
			return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)));
		}		
	} else if ( accuracy == 1 ) {
		if ( gas_id == GAS_HYDROGEN ) {
			return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)));
		} else if ( gas_id == GAS_LPG ) {
			return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)));
		} else if ( gas_id == GAS_METHANE ) {
			return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)));
		} else if ( gas_id == GAS_CARBON_MONOXIDE ) {
			return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)));
		} else if ( gas_id == GAS_ALCOHOL ) {
			return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)));
		} else if ( gas_id == GAS_SMOKE ) {
			return (pow(10,(-0.976*pow((log10(rs_ro_ratio)), 2) - 2.018*(log10(rs_ro_ratio)) + 3.617)));
		} else if ( gas_id == GAS_PROPANE ) {
			return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)));
		}
	}		
	return 0;
}

void printVals()
{
	double val = MQResistance()/Ro;
	Serial.print("HYDROGEN:");
	Serial.print(MQGetGasPercentage(val, GAS_HYDROGEN));
	Serial.print( "ppm" );
	Serial.print(" ");
	Serial.print("LPG:");
	Serial.print(MQGetGasPercentage(val, GAS_LPG));
	Serial.print( "ppm" );
	Serial.print(" ");
	Serial.print("METHANE:");
	Serial.print(MQGetGasPercentage(val, GAS_METHANE));
	Serial.print( "ppm" );
	Serial.print(" ");
	Serial.print("CARBON_MONOXIDE:");
	Serial.print(MQGetGasPercentage(val, GAS_CARBON_MONOXIDE));
	Serial.print( "ppm" );
	Serial.print(" ");
	Serial.print("ALCOHOL:");
	Serial.print(MQGetGasPercentage(val, GAS_ALCOHOL));
	Serial.print( "ppm" );
	Serial.print(" ");
	Serial.print("SMOKE:");
	Serial.print(MQGetGasPercentage(val, GAS_SMOKE));
	Serial.print( "ppm" );
	Serial.print(" ");
	Serial.print("PROPANE:");
	Serial.print(MQGetGasPercentage(val, GAS_PROPANE));
	Serial.print( "ppm" );
	Serial.print("\n");
}

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 8989; // local port to listen on

WiFiUDP Udp;
ESP8266WebServer server(80);

boolean wifiConnected = false;
boolean udpConnected = false;

typedef struct __attribute__((packed)) {
	int32_t data_send_period;
	uint16_t checksum;
} settings_t;

settings_t settings = {.data_send_period = 5000};

typedef struct {
	time_t ts;
	double voltage;
	double rs_air;
	double r0;
	double res;
} val_t;

val_t cur_data = {0, 0};

char htmlbuf[120];

void load_settings()
{
	// Restore from EEPROM, check the checksum matches
	settings_t s;
	uint8_t *ptr = reinterpret_cast<uint8_t *>(&s);
	EEPROM.begin(sizeof(s));
	uint16_t sum = 0x1234;
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
	s.checksum = 0x1234;
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
			"<input type=\"submit\" value=\"Submit\"></input>"
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

double calc_rs_air(double sensorVoltage)
{
	double RS_air; //Define variable for sensor resistance
	RS_air = (5/sensorVoltage)-1; //Calculate RS in fresh air
	return RS_air;
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
	if(wifiConnected && udpConnected) {
		cur_data.voltage = readVoltage();
		cur_data.rs_air = calc_rs_air(cur_data.voltage);
		cur_data.r0 = cur_data.rs_air/RO_CLEAN_AIR_FACTOR_MQ2;
		cur_data.res = MQResistance();
		cur_data.ts = time(NULL);
		Serial.print("V=");
		Serial.println(cur_data.voltage);
		Serial.print("R0=");
		Serial.println(cur_data.r0);
		send_udp_string("{\"sensor_id\":\"" + sensor_id + "\",\"ts\":" + String(cur_data.ts) +
			",\"rawval\":" + String(cur_data.voltage) + ", \"r0\":" + String(cur_data.r0) +
			",\"rs_air\":" + String(cur_data.rs_air) + ",\"res\":" + String(cur_data.res) + "}");
		double val = MQResistance()/Ro;
		String js = "{";
		js += "\"HYDROGEN\":" + String(MQGetGasPercentage(val, GAS_HYDROGEN));
		js += ",\"LPG\":" + String(MQGetGasPercentage(val, GAS_LPG));
		js += ",\"METHANE\":" + String(MQGetGasPercentage(val, GAS_METHANE));
		js += ",\"CARBON_MONOXIDE\":" + String(MQGetGasPercentage(val, GAS_CARBON_MONOXIDE));
		js += ",\"ALCOHOL\":" + String(MQGetGasPercentage(val, GAS_ALCOHOL));
		js += ",\"SMOKE\":" + String(MQGetGasPercentage(val, GAS_SMOKE));
		js += ",\"PROPANE\":" + String(MQGetGasPercentage(val, GAS_PROPANE));
		js += "}";
		send_udp_string(js);
		blinker.blink(3);
	}
}

void make_ticker()
{
	ticker.detach();
	ticker.attach_ms(settings.data_send_period, send_data);
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
			make_ticker();
			message = "Success";
		} else {
			message = "Error in form";
		}
		server.send(200, "text/html", wrap_html(home_link + "<br>" + message + "<br><a href=\"/config.html\">Config</a>"));
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
		server.send(200, "text/html", wrap_html(htmlbuf, reload_script));
		blinker.blink(3);
	});
	server.on("/config.html", []() {
		server.send(200, "text/html", wrap_html(home_link + postForm[0] + settings.data_send_period + postForm[1]));
	});
	server.on("/write_settings.php", handleForm);
	server.onNotFound(handleNotFound);
	server.begin();
	Serial.println("HTTP server started");
	make_ticker();
	digitalWrite(LED_PIN, LOW);
	Ro = MQCalibration(); //Calibrating the sensor. Please make sure the sensor is in clean air
	Serial.print("Calibration is done...\n");
	Serial.print("Ro=");
	Serial.print(Ro);
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
	delay(10);
}

