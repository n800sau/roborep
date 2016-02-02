#include <IRremoteESP8266.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include "config.h"

#define HOSTNAME "irproxy"

const char* ssid     = SSID;
const char* password = PASSWORD;


int RECV_PIN = 2; //an IR detector/demodulator is connected to GPIO pin 2

IRrecv irrecv(RECV_PIN);

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 12345;      // local port to listen on

void  setup ( )
{
	Serial.begin(115200);
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
	}

	irrecv.enableIRIn();  // Start the receiver

//	Udp.beginMulticast(WiFi.localIP(),  ipMulti, portMulti);
//	Serial.print("Udp Multicast server started at : ");
//	Serial.print(ipMulti);
//	Serial.print(":");
//	Serial.println(portMulti);
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

//+=============================================================================
// The repeating section of the code
//
void  loop ( )
{
	decode_results  results;        // Somewhere to store the results

	if (irrecv.decode(&results)) {  // Grab an IR code

		// send the ir data
		Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
		Udp.print(encoding(&results));
		Udp.endPacket();

		dumpInfo(&results);           // Output the results
		dumpRaw(&results);            // Output the results in RAW format
		dumpCode(&results);           // Output the results as source code
		Serial.println("");           // Blank line between entries
		irrecv.resume();              // Prepare for the next value
//	} else {
		// send no data message
//		Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
//		Udp.print("No data");
//		Udp.endPacket();
	}
}
