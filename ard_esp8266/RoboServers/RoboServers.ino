#include "rest_service.h"
#include "www_service.h"
#include "t2s_service.h"
#include "uart_utils.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>

const char* ssid = "Slow Internet Connection";
const char* password = "1,tpGfhjkz2";

static Ticker camticker;

void setup() {
	Serial1.begin(115200);

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
//	WiFi.softAP("ESP Monster 96", "n42n3hofdS");

	Serial1.print("\nConnecting to ");
	Serial1.println(ssid);

	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
	if(i == 21){
		Serial1.print("Could not connect to"); Serial1.println(ssid);
		while(1) delay(500);
	}

	setupT2Sservice();
	Serial1.println("Telnet server started");

	setupWWWservice();
	Serial1.println ( "HTTP server started" );

	setupRESTservice();
	Serial1.println("REST server started");

	Serial1.println(WiFi.localIP());

}

void loop() {
	// handle WEB
	handleWWWservice();

	// handle telnet 2 UART
	handleT2Sservice();

	// handle REST calls
	handleRESTservice();

}
