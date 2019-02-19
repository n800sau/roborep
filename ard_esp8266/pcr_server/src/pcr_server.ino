/*
	upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
	or you can upload the contents of a folder if you CD in that folder and run the following command:
	for file in `ls -A1`; do curl -F "file=@$PWD/$file" webservos.local/edit; done

	access the sample web page at http://webservos.local
	edit the page by going to http://webservos.local/edit
*/
#include "fs_webserver_app.h"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <FS.h>

FSWifiWebServerApp app("pcr_server");

void readSettings()
{
	String path = "/settings.json";
	if(SPIFFS.exists(path)) {
		File file = SPIFFS.open(path, "r");
		app.webserver.streamFile(file, "text/json");
		file.close();
	} else {
		app.webserver.send(200, "text/json", "[]");
	}
}

void setup()
{
	//WIFI INIT
//	DBG_OUTPUT_PORT.printf("Connecting to %s\n", ssid);
//	if (String(WiFi.SSID()) != String(ssid)) {
//		WiFi.mode(WIFI_STA);
//		WiFi.begin(ssid, password);
//	}

	app.setup();
	app.webserver.on("/settings.json", HTTP_GET, readSettings);
}

void loop(void)
{
	app.loop();
}
