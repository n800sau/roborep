// get data from DHT11 nrf pipe and send it to THE SERVER

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <HttpClient.h>
#include <Ticker.h>
#include <Time.h>

const char *dest_server = "192.168.2.80";
const int dest_server_port = 5580;
const char *dest_path = "/sensors/accept.php";

#include "config.h"
const char* ssid = SSID;
const char* password = PASSWORD;

const char *ap_ssid = AP_SSID;
const char *ap_password = AP_PASSWORD;

#define LED_PIN 4

ESP8266WebServer server(80);


// NTP stuff

unsigned int ntpLocalPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

Ticker blinker;

volatile int blinking_times = 0;
volatile int blinking_state = 0;

// attention! delays in tickers crash
void blinking()
{
	if(blinking_times > 0) {
		if(blinking_state) {
			Serial.println("led off");
			digitalWrite(LED_PIN, LOW);
			blinking_times--;
		} else {
			Serial.println("led on");
			digitalWrite(LED_PIN, HIGH);
		}
		blinking_state = !blinking_state;
	}
}

void blink(int times=1)
{
	blinking_times = times;
	blinking_state = 0;
}

void handleRoot() {
	digitalWrite(LED_PIN, HIGH);
	server.send(200, "text/plain", "hello from " SSID "!");
	digitalWrite(LED_PIN, LOW);
}

void handleDHT11()
{
	Serial.println("dht11 handler start");
	String t = server.arg("T");
	String h = server.arg("H");
	String v = server.arg("V");
	String id = server.arg("ID");
	String time = server.arg("TIME");
	time_t itime = time.toInt();
	if(t == "" || h == "" || v == "") {
		server.send(500, "text/plain", "No data received");
		blink(3);
	} else {
		Serial.print("Data timestamp:");
		Serial.println(String(year(itime)) + "-" + String(month(itime)) + "-" + String(day(itime)) + " " +
			String(hour(itime)) + ":" + String(minute(itime)) + ":" + String(second(itime)));
		if(send_dht11_data(id, t, h, v, time)) {
			server.send(200, "text/plain", "Ok");
			blink(1);
		} else {
			server.send(500, "text/plain", "Destination connection failed");
			Serial.println("Destination connection failed");
			blink(3);
		}
	}
	Serial.println("dht11 handler end");
}

void handleEpoch()
{
	unsigned long epoch = requestTime();
//	Serial.print("Sending back epoch: ");
//	Serial.println(String(epoch));
	server.send(200, "text/plain", String(epoch));
}

void handleNotFound()
{
	digitalWrite(LED_PIN, LOW);
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
	digitalWrite(LED_PIN, LOW);
}


void setup()
{

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	printf("\r\nRF24 gate\r\n");

	WiFi.softAP(ap_ssid, ap_password, 9);
	WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 0));

	// Connect to WiFi network
	WiFi.begin(ssid, password);
	Serial.print("\n\r \n\rWorking to connect");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("DHT Weather Reading Server");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.println("IP addresses: ");
	Serial.println(WiFi.localIP());
	Serial.println(WiFi.softAPIP());

	blink(1);

	if (MDNS.begin(ap_ssid)) {
		Serial.print("mDNS responder started as ");
		Serial.println(ap_ssid);
		MDNS.addService("http", "tcp", 80);
	} else {
		Serial.println("Error setting up mDNS responder!");
	}

	server.on("/", handleRoot);

	server.on("/dht11", HTTP_POST, handleDHT11);

	server.on("/epoch", HTTP_GET, handleEpoch);

	server.onNotFound(handleNotFound);

	server.begin();
	Serial.println("HTTP server started");

	blinker.attach(0.3, blinking);

	Serial.println("Starting UDP");
	udp.begin(ntpLocalPort);
	Serial.print("Local port: ");
	Serial.println(udp.localPort());

}

bool not_available = false;

void loop() {

	server.handleClient();
//	Serial.print("Light:");
//	Serial.println(analogRead(A0));

} // Loop


bool send_dht11_data(String id, String temp_c, String humidity, String v, String timestamp)
{
	bool rs = false;
	WiFiClient client;
	HttpClient http(client);
	String postStr =
		"TIME=" + timestamp +
		"&T=" + temp_c +
		"&H=" + humidity +
		"&V=" + v +
		"&ID=" + id;

	Serial.println(postStr);
	http.beginRequest();
	int err = http.post(dest_server, dest_server_port, dest_path);
	if (err == 0)
	{
		http.sendHeader("Content-Type", "application/x-www-form-urlencoded");
		http.sendHeader("Content-Length", postStr.length());
		http.println(postStr);
		http.endRequest();
		err = http.responseStatusCode();
		if (err >= 0)
		{
			Serial.print("Got status code: ");
			Serial.println(err);
			rs = err < 300;
		}
		else
		{
			Serial.print("Getting response failed: ");
			Serial.println(err);
		}
	} else {
		blink(2);
		Serial.print("Connect failed: ");
		Serial.println(err);
	}
	http.stop();
	return rs;
}

unsigned long requestTime()
{
	unsigned long epoch = 0;

	//get a random server from the pool
	WiFi.hostByName(ntpServerName, timeServerIP); 

	sendNTPpacket(timeServerIP); // send an NTP packet to a time server
	// wait to see if a reply is available
	delay(1000);
	
	int cb = udp.parsePacket();
	if (!cb) {
		Serial.println("no packet yet");
	}
	else {
//		Serial.print("packet received, length=");
//		Serial.println(cb);
		// We've received a packet, read the data from it
		udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

		//the timestamp starts at byte 40 of the received packet and is four bytes,
		// or two words, long. First, esxtract the two words:

		unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
		unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
		// combine the four bytes (two words) into a long integer
		// this is NTP time (seconds since Jan 1 1900):
		unsigned long secsSince1900 = highWord << 16 | lowWord;
//		Serial.print("Seconds since Jan 1 1900 = " );
//		Serial.println(secsSince1900);

		// now convert NTP time into everyday time:
		// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
		const unsigned long seventyYears = 2208988800UL;
		// subtract seventy years:
		epoch = secsSince1900 - seventyYears;
		// print Unix time:
//		Serial.print("Unix time = ");
//		Serial.println(epoch);


		// print the hour, minute and second:
		Serial.print("The UTC time is ");			 // UTC is the time at Greenwich Meridian (GMT)
		Serial.print((epoch	% 86400L) / 3600); // print the hour (86400 equals secs per day)
		Serial.print(':');
		if ( ((epoch % 3600) / 60) < 10 ) {
			// In the first 10 minutes of each hour, we'll want a leading '0'
			Serial.print('0');
		}
		Serial.print((epoch	% 3600) / 60); // print the minute (3600 equals secs per minute)
		Serial.print(':');
		if ( (epoch % 60) < 10 ) {
			// In the first 10 seconds of each minute, we'll want a leading '0'
			Serial.print('0');
		}
		Serial.println(epoch % 60); // print the second
	}
	return epoch;
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
//	Serial.println("sending NTP packet...");
	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)
	packetBuffer[0] = 0b11100011;	 // LI, Version, Mode
	packetBuffer[1] = 0;		 // Stratum, or type of clock
	packetBuffer[2] = 6;		 // Polling Interval
	packetBuffer[3] = 0xEC;	// Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12]	= 49;
	packetBuffer[13]	= 0x4E;
	packetBuffer[14]	= 49;
	packetBuffer[15]	= 52;

	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp:
	udp.beginPacket(address, 123); //NTP requests are to port 123
	udp.write(packetBuffer, NTP_PACKET_SIZE);
	udp.endPacket();
}
