/*
	upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
	or you can upload the contents of a folder if you CD in that folder and run the following command:
	for file in `ls -A1`; do curl -F "file=@$PWD/$file" webservos.local/edit; done

	access the sample web page at http://webservos.local
	edit the page by going to http://webservos.local/edit
*/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <IRremoteESP8266.h>

#include <ArduinoJson.h>
#include <EEPROM.h>

#include <DNSServer.h>
#include <WiFiManager.h>

#define EEPROM_MARKER 0x4A

#define DBG_OUTPUT_PORT Serial

#include "config.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* host = "web2ir";

ESP8266WebServer server(80);
//holds the current upload
File fsUploadFile;

const int IROUT_PIN = 15;
IRsend irsend(IROUT_PIN);

//format bytes
String formatBytes(size_t bytes) {
	if (bytes < 1024) {
		return String(bytes) + "B";
	} else if (bytes < (1024 * 1024)) {
		return String(bytes / 1024.0) + "KB";
	} else if (bytes < (1024 * 1024 * 1024)) {
		return String(bytes / 1024.0 / 1024.0) + "MB";
	} else {
		return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
	}
}

String getContentType(String filename) {
	if (server.hasArg("download")) {
		return "application/octet-stream";
	} else if (filename.endsWith(".htm")) {
		return "text/html";
	} else if (filename.endsWith(".html")) {
		return "text/html";
	} else if (filename.endsWith(".css")) {
		return "text/css";
	} else if (filename.endsWith(".js")) {
		return "application/javascript";
	} else if (filename.endsWith(".png")) {
		return "image/png";
	} else if (filename.endsWith(".gif")) {
		return "image/gif";
	} else if (filename.endsWith(".jpg")) {
		return "image/jpeg";
	} else if (filename.endsWith(".ico")) {
		return "image/x-icon";
	} else if (filename.endsWith(".xml")) {
		return "text/xml";
	} else if (filename.endsWith(".pdf")) {
		return "application/x-pdf";
	} else if (filename.endsWith(".zip")) {
		return "application/x-zip";
	} else if (filename.endsWith(".gz")) {
		return "application/x-gzip";
	}
	return "text/plain";
}

bool handleFileRead(String path) {
	DBG_OUTPUT_PORT.println("handleFileRead: " + path);
	if (path.endsWith("/")) {
		path += "index.html";
	}
	String contentType = getContentType(path);
	String pathWithGz = path + ".gz";
	if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
		if (SPIFFS.exists(pathWithGz)) {
			path += ".gz";
		}
		File file = SPIFFS.open(path, "r");
		server.streamFile(file, contentType);
		file.close();
		return true;
	}
	return false;
}

void handleFileUpload() {
	if (server.uri() != "/edit") {
		return;
	}
	HTTPUpload& upload = server.upload();
	if (upload.status == UPLOAD_FILE_START) {
		String filename = upload.filename;
		if (!filename.startsWith("/")) {
			filename = "/" + filename;
		}
		DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
		fsUploadFile = SPIFFS.open(filename, "w");
		filename = String();
	} else if (upload.status == UPLOAD_FILE_WRITE) {
		//DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
		if (fsUploadFile) {
			fsUploadFile.write(upload.buf, upload.currentSize);
		}
	} else if (upload.status == UPLOAD_FILE_END) {
		if (fsUploadFile) {
			fsUploadFile.close();
		}
		DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
	}
}

void handleFileDelete() {
	if (server.args() == 0) {
		return server.send(500, "text/plain", "BAD ARGS");
	}
	String path = server.arg(0);
	DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
	if (path == "/") {
		return server.send(500, "text/plain", "BAD PATH");
	}
	if (!SPIFFS.exists(path)) {
		return server.send(404, "text/plain", "FileNotFound");
	}
	SPIFFS.remove(path);
	server.send(200, "text/plain", "");
	path = String();
}

void handleFileCreate() {
	if (server.args() == 0) {
		return server.send(500, "text/plain", "BAD ARGS");
	}
	String path = server.arg(0);
	DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
	if (path == "/") {
		return server.send(500, "text/plain", "BAD PATH");
	}
	if (SPIFFS.exists(path)) {
		return server.send(500, "text/plain", "FILE EXISTS");
	}
	File file = SPIFFS.open(path, "w");
	if (file) {
		file.close();
	} else {
		return server.send(500, "text/plain", "CREATE FAILED");
	}
	server.send(200, "text/plain", "");
	path = String();
}

void handleFileList() {
	if (!server.hasArg("dir")) {
		server.send(500, "text/plain", "BAD ARGS");
		return;
	}

	String path = server.arg("dir");
	DBG_OUTPUT_PORT.println("handleFileList: " + path);
	Dir dir = SPIFFS.openDir(path);
	path = String();

	String output = "[";
	while (dir.next()) {
		File entry = dir.openFile("r");
		if (output != "[") {
			output += ',';
		}
		bool isDir = false;
		output += "{\"type\":\"";
		output += (isDir) ? "dir" : "file";
		output += "\",\"name\":\"";
		output += String(entry.name()).substring(1);
		output += "\"}";
		entry.close();
	}

	output += "]";
	server.send(200, "text/json", output);
}

void settings_from_ROM() {
	int addr = 1;
}

void settings_to_ROM() {
	int addr = 1;
	EEPROM.write(0, EEPROM_MARKER);
	EEPROM.commit();
}

void setup(void)
{
	DBG_OUTPUT_PORT.begin(115200);
	DBG_OUTPUT_PORT.print("\n");
	DBG_OUTPUT_PORT.setDebugOutput(true);
	SPIFFS.begin();
	{
		Dir dir = SPIFFS.openDir("/");
		while (dir.next()) {
			String fileName = dir.fileName();
			size_t fileSize = dir.fileSize();
			DBG_OUTPUT_PORT.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
		}
		DBG_OUTPUT_PORT.printf("\n");
	}


	EEPROM.begin(512);
	if(EEPROM.read(0) != EEPROM_MARKER) {
		settings_to_ROM();
	} else {
		settings_from_ROM();
	}

//	WiFiManager wifiManager;

	//reset saved settings
	//wifiManager.resetSettings();

	//WIFI INIT
	DBG_OUTPUT_PORT.printf("Connecting to %s\n", ssid);
	if (String(WiFi.SSID()) != String(ssid)) {
		WiFi.mode(WIFI_STA);
		WiFi.begin(ssid, password);
	}
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		DBG_OUTPUT_PORT.print(".");
	}

	//set custom ip for portal
	//wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

	//fetches ssid and pass from eeprom and tries to connect
	//if it does not connect it starts an access point with the specified name
/*	if (!wifiManager.autoConnect(host)) {
		Serial.println("failed to connect and hit timeout");
		delay(3000);
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(5000);
	}
*/

	DBG_OUTPUT_PORT.println();
	DBG_OUTPUT_PORT.print("Connected! IP address: ");
	DBG_OUTPUT_PORT.println(WiFi.localIP());

	if(!MDNS.begin(host)) {
		Serial.println("Error setting up MDNS responder!");
	}
	MDNS.addService("http", "tcp", 80);
	DBG_OUTPUT_PORT.print("Open http://");
	DBG_OUTPUT_PORT.print(host);
	DBG_OUTPUT_PORT.println(".local/edit to see the file browser");


	//SERVER INIT
	//list directory
	server.on("/list", HTTP_GET, handleFileList);
	//load editor
	server.on("/edit", HTTP_GET, []() {
		if (!handleFileRead("/edit.htm")) {
			server.send(404, "text/plain", "FileNotFound");
		}
	});
	//create file
	server.on("/edit", HTTP_PUT, handleFileCreate);
	//delete file
	server.on("/edit", HTTP_DELETE, handleFileDelete);
	//first callback is called after the request has ended with all parsed arguments
	//second callback handles file uploads at that location
	server.on("/edit", HTTP_POST, []() {
		server.send(200, "text/plain", "");
	}, handleFileUpload);

	//called when the url is not defined here
	//use it to load content from SPIFFS
	server.onNotFound([]() {
		if (!handleFileRead(server.uri())) {
			server.send(404, "text/plain", "FileNotFound");
		}
	});

	//get heap status, analog input value and all GPIO statuses in one json call
	server.on("/all", HTTP_GET, []() {
		String json = "{";
		json += "\"heap\":" + String(ESP.getFreeHeap());
		json += ", \"analog\":" + String(analogRead(A0));
		json += ", \"gpio\":" + String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
		json += "}";
		server.send(200, "text/json", json);
		json = String();
	});

	server.on("/store_settings", HTTP_POST, []() {
		// store settings here
		StaticJsonBuffer<500> jsonBuffer;
		String js = server.arg("plain");
		JsonObject& root = jsonBuffer.parseObject(js);
		if (!root.success()) {
			Serial.println("parseObject() failed");
			server.send(400, "text/plain", "Bad request");
		} else {
			settings_to_ROM();
			server.send(200, "text/plain", "Settings stored");
		}
	});

	server.on("/store_values", HTTP_POST, []() {
		// set servo values here
	StaticJsonBuffer<500> jsonBuffer;
	String js = server.arg("plain");
	JsonObject& root = jsonBuffer.parseObject(js);
	if (!root.success()) {
		Serial.println("parseObject() failed");
		server.send(400, "text/plain", "Bad request");
	} else {
		server.send(200, "text/plain", "Values stored");
	}
	});

	server.on("/settings", HTTP_GET, []() {
		StaticJsonBuffer<500> jsonBuffer;
		String js = server.arg("plain");
		JsonObject& root = jsonBuffer.createObject();
		JsonArray& data = root.createNestedArray("settings");
		String rs;
		root.printTo(rs);
		DBG_OUTPUT_PORT.println(rs);
		server.send(200, "application/json", rs);
	});

	server.begin();
	DBG_OUTPUT_PORT.println("HTTP server started");

	irsend.begin();

	server.on("/tv_off", HTTP_GET, []() {
		int bits = 12;
		int ir_code = 0xA90;
		irsend.sendSony(ir_code, (bits > 0) ? bits : 12);
	});
}

void loop(void)
{
	MDNS.update();
	server.handleClient();
}
