#ifndef __FS_WEBSERVER_H
#define __FS_WEBSERVER_H

/*
	upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
	or you can upload the contents of a folder if you CD in that folder and run the following command:
	for file in `ls -A1`; do curl -F "file=@$PWD/$file" webservos.local/edit; done

	access the sample web page at http://webservos.local
	edit the page by going to http://webservos.local/edit
*/

#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>

#include <Servo.h>
#include <ArduinoJson.h>

#include <DNSServer.h>
#include <WiFiManager.h>

class FSWifiWebServerApp {

	private:
		File fsUploadFile;

	protected:
		String host;
		ESP8266WebServer webserver;

	public:

		FSWifiWebServerApp(String host, int port=80);
		String formatBytes(size_t bytes);
		String getContentType(String filename);
		bool handleFileRead(String path);
		void handleFileUpload();
		void handleFileDelete();
		void handleFileCreate();
		void handleFileList();
		virtual void setup();
		virtual void update_process();
		virtual void loop();

};

#endif //__FS_WEBSERVER_H
