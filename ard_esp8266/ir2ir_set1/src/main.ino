#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <IRremoteESP8266.h>

#include "nec_codes.h"

#include "config.h"

// needed to avoid link error on ram check
extern "C"
{
#include "user_interface.h"
}

ADC_MODE(ADC_VCC);

#define HOSTNAME "ir2ir"

ESP8266WebServer server(80);
WiFiClient client;

const int RED_PIN = 12;
const int GREEN_PIN = 5;
const int YELLOW_PIN = 14;

const int IROUT_PIN = 15;

const int IRIN_PIN = 13; //an IR detector/demodulator is connected to GPIO pin 13

const int LED_COUNT = 4;

const char *led_names[LED_COUNT] = {
	"red_led",
	"green_led",
	"yellow_led",
	"ir_led"
};

const int led_pins[LED_COUNT] {
	RED_PIN,
	GREEN_PIN,
	YELLOW_PIN,
	IROUT_PIN
};

const int IR_TYPE_COUNT = 2;

const char *ir_type_names[IR_TYPE_COUNT] = {
	"sony",
	"nec"
};

const decode_type_t ir_types[IR_TYPE_COUNT] = {
	SONY,
	NEC
};

const char* ssid     = SSID;
const char* password = PASSWORD;

float pfVcc;
IRrecv irrecv(IRIN_PIN);
IRsend irsend(IROUT_PIN);

JsonObject& prepareResponse(JsonBuffer& jsonBuffer)
{
	JsonObject& root = jsonBuffer.createObject();
	root["Systemv"] = pfVcc/1000;
	return root;
}

void blink(int pin, int times=5)
{
	Serial.print("BLINK ");
	Serial.println(pin);
	for(int i=0; i<times; i++) {
		digitalWrite(pin, HIGH);
		delay(100);
		digitalWrite(pin, LOW);
		if(i < times) {
			delay(200);
		}
	}
}

bool processCmd(JsonObject& root, bool check_only)
{
	bool rs = false;
	String method = (const char *)root["method"];
	if(check_only) {
		Serial.print("Method:");
		Serial.println(method);
		root["params"].prettyPrintTo(Serial);
		Serial.println();
	}
	if(method == "blink") {
		String led = root["params"]["led"];
		int led_pin;
		for(int i=0; i<LED_COUNT; i++) {
			if(led == led_names[i]) {
				led_pin = led_pins[i];
				rs = true;
				break;
			}
		}
		if(rs) {
			int times = root["params"]["times"];
			if(check_only) {
				Serial.print("Led:");
				Serial.println(led);
				Serial.print("Pin:");
				Serial.println(led_pin);
				Serial.print("Times:");
				Serial.println(times);
			} else {
				Serial.println("Executing");
				blink(led_pin, times ? times : 3);
			}
		}
	} else if(method == "ir_send") {
		String ir_type_name = root["params"]["type"];
		decode_type_t ir_type;
		int ir_code = root["params"]["code"];
		if(!ir_code) {
			String hexval = root["params"]["code"];
			ir_code = strtol(hexval.c_str(), NULL, 16);
		}
		if(ir_code) {
			for(int i=0; i<IR_TYPE_COUNT; i++) {
				if(ir_type_name == ir_type_names[i]) {
					ir_type = ir_types[i];
					rs = true;
					break;
				}
			}
		}
		if(rs) {
			if(check_only) {
				Serial.print("Type:");
				Serial.println(ir_type_name);
				Serial.print("Code: 0x");
				Serial.println(ir_code, HEX);
			} else {
				Serial.println("Executing");
				switch(ir_type) {
					case SONY:
						irsend.sendSony(ir_code, 12);
						break;
					case NEC:
						irsend.sendNEC(ir_code, 32);
						break;
				}
			}
		}
	}
	return rs;
}

void handleRoot()
{
	Serial.println("Request:");
	String js = server.arg("plain");
	Serial.println(js);

	StaticJsonBuffer<500> jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(js);
	if (!root.success()) {
		Serial.println("parseObject() failed");
		server.send(400, "text/plain", "Bad request");
	} else {
		if(processCmd(root, true)) {
			pfVcc = ESP.getVcc();
			StaticJsonBuffer<500> jsonBuffer;
			JsonObject& json = prepareResponse(jsonBuffer);
			String content;
			json.prettyPrintTo(content);
			server.send(200, "application/json-rpc", content);
			processCmd(root, false);
		} else {
			Serial.println("Bad parameters");
			server.send(400, "text/plain", "Bad request parameters");
		}
	}
}

void setup()
{
	Serial.begin(115200);

	pinMode(RED_PIN, OUTPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(YELLOW_PIN, OUTPUT);
	pinMode(IROUT_PIN, OUTPUT);

	WiFi.begin(ssid, password);
	Serial.println("");
	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	if (MDNS.begin(HOSTNAME)) {
		Serial.println("MDNS responder started");
		Serial.println("URL: http://" HOSTNAME ".local");
		MDNS.addService("http", "tcp", 80);
	}

	server.on("/", handleRoot);
	server.begin();
	Serial.println("HTTP server started");
	irrecv.enableIRIn();
	irsend.begin();
}

//+=============================================================================
// Display IR code
//
void  ircode (decode_results *results)
{
	// Panasonic has an Address
	if (results->decode_type == PANASONIC) {
		Serial.print(results->panasonicAddress, HEX);
		Serial.print(":");
	}

	// Print Code
	Serial.print(results->value, HEX);
}

//+=============================================================================
// Display encoding type
//
const char *encoding (decode_results *results)
{
	const char *rs = NULL;
	switch (results->decode_type) {
		default:                                 break ;
		case UNKNOWN:      rs = "UNKNOWN";       break ;
		case NEC:          rs = "NEC";           break ;
		case SONY:         rs = "SONY";          break ;
		case RC5:          rs = "RC5";           break ;
		case RC6:          rs = "RC6";           break ;
		case DISH:         rs = "DISH";          break ;
		case SHARP:        rs = "SHARP";         break ;
		case JVC:          rs = "JVC";           break ;
		case SANYO:        rs = "SANYO";         break ;
		case MITSUBISHI:   rs = "MITSUBISHI";    break ;
		case SAMSUNG:      rs = "SAMSUNG";       break ;
		case LG:           rs = "LG";            break ;
		case WHYNTER:      rs = "WHYNTER";       break ;
		case AIWA_RC_T501: rs = "AIWA_RC_T501";  break ;
		case PANASONIC:    rs = "PANASONIC";     break ;
	}
	if(rs) Serial.print(rs);
	return rs;
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpInfo (decode_results *results)
{
	// Show Encoding standard
	Serial.print("Encoding  : ");
	encoding(results);
	Serial.println("");

	// Show Code & length
	Serial.print("Code      : ");
	ircode(results);
	Serial.print(" (");
	Serial.print(results->bits, DEC);
	Serial.println(" bits)");
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpRaw (decode_results *results)
{
	// Print Raw data
	Serial.print("Timing[");
	Serial.print(results->rawlen-1, DEC);
	Serial.println("]: ");

	for (int i = 1;  i < results->rawlen;  i++) {
		unsigned long  x = results->rawbuf[i] * USECPERTICK;
		if (!(i & 1)) {  // even
			Serial.print("-");
			if (x < 1000)  Serial.print(" ") ;
			if (x < 100)   Serial.print(" ") ;
			Serial.print(x, DEC);
		} else {  // odd
			Serial.print("     ");
			Serial.print("+");
			if (x < 1000)  Serial.print(" ") ;
			if (x < 100)   Serial.print(" ") ;
			Serial.print(x, DEC);
			if (i < results->rawlen-1) Serial.print(", "); //',' not needed for last one
		}
		if (!(i % 8))  Serial.println("");
	}
	Serial.println("");                    // Newline
}

//+=============================================================================
// Dump out the decode_results structure.
//
void  dumpCode (decode_results *results)
{
	// Start declaration
	Serial.print("unsigned int  ");          // variable type
	Serial.print("rawData[");                // array name
	Serial.print(results->rawlen - 1, DEC);  // array size
	Serial.print("] = {");                   // Start declaration

	// Dump data
	for (int i = 1;  i < results->rawlen;  i++) {
		Serial.print(results->rawbuf[i] * USECPERTICK, DEC);
		if ( i < results->rawlen-1 ) Serial.print(","); // ',' not needed on last one
		if (!(i & 1))  Serial.print(" ");
	}

	// End declaration
	Serial.print("};");  //

	// Comment
	Serial.print("  // ");
	encoding(results);
	Serial.print(" ");
	ircode(results);

	// Newline
	Serial.println("");

	// Now dump "known" codes
	if (results->decode_type != UNKNOWN) {

		// Some protocols have an address
		if (results->decode_type == PANASONIC) {
			Serial.print("unsigned int  addr = 0x");
			Serial.print(results->panasonicAddress, HEX);
			Serial.println(";");
		}

		// All protocols have data
		Serial.print("unsigned int  data = 0x");
		Serial.print(results->value, HEX);
		Serial.println(";");
	}
}

void loop()
{
	decode_results  results;        // Somewhere to store the results

	if (irrecv.decode(&results)) {  // Grab an IR code
		digitalWrite(GREEN_PIN, HIGH);
		if(results.decode_type == NEC) {
			switch(results.value) {
				case TURN_OFF:
					ESP.reset();
					break;
				default:
					break;
			}
		} else {
			Serial.println("Unknown encoding");
		}
		dumpInfo(&results);           // Output the results
		dumpRaw(&results);            // Output the results in RAW format
		dumpCode(&results);           // Output the results as source code
		Serial.println("");           // Blank line between entries

		irrecv.resume();              // Prepare for the next value
		digitalWrite(GREEN_PIN, LOW);
	}

	server.handleClient();
}
