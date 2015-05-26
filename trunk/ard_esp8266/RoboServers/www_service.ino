#include "www_service.h"
#include <ESP8266WebServer.h>

const int led = 13;

static ESP8266WebServer wserver(80);

static void ICACHE_FLASH_ATTR handleRoot() {
	digitalWrite ( led, 1 );
	char temp[400];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf ( temp, 400,

"<html>\
	<head>\
		<meta http-equiv='refresh' content='5'/>\
		<title>ESP8266 Demo</title>\
		<style>\
			body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
		</style>\
	</head>\
	<body>\
		<h1>Hello from ESP8266!</h1>\
		<p>Uptime: %02d:%02d:%02d</p>\
		<img src=\"/test.svg\" />\
	</body>\
</html>",

		hr, min % 60, sec % 60
	);
	wserver.send ( 200, "text/html", temp );
	digitalWrite ( led, 0 );

}

static void ICACHE_FLASH_ATTR drawGraph() {
	String out = "";
	char temp[100];
	out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
	out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
	out += "<g stroke=\"black\">\n";
	int y = rand() % 130;
	for (int x = 10; x < 390; x+= 10) {
		int y2 = rand() % 130;
		sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
		out += temp;
		y = y2;
	}
	out += "</g>\n</svg>\n";

	wserver.send ( 200, "image/svg+xml", out);
}

void ICACHE_FLASH_ATTR setupWWWservice()
{
	wserver.on ( "/", handleRoot );
	wserver.on ( "/test.svg", drawGraph );
	wserver.begin();
}

void ICACHE_FLASH_ATTR handleWWWservice()
{
	wserver.handleClient();
}

