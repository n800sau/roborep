#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <SdFat.h>

#include "config.h"

#define DBG_OUTPUT_PORT Serial

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* host = "esp8266sd";

ESP8266WebServer server(80);

using namespace sdfat;
using sdfat::File;
using sdfat::FatFile;
using sdfat::FatFileSystem;

#define SD_CHIP_SELECT 15
//#define SD_CHIP_SELECT SS

static bool hasSD = false;
File uploadFile;
SdFat SD;

char namebuf[256];


void returnOK() {
	server.send(200, "text/plain", "");
}

void returnFail(String msg) {
	server.send(500, "text/plain", msg + "\r\n");
}

bool loadFromSdCard(String path) {
	String dataType = "text/plain";
	if (path.endsWith("/")) {
		path += "index.html";
	}

	if (path.endsWith(".src")) {
		path = path.substring(0, path.lastIndexOf("."));
	} else if (path.endsWith(".htm")) {
		dataType = "text/html";
	} else if (path.endsWith(".css")) {
		dataType = "text/css";
	} else if (path.endsWith(".js")) {
		dataType = "application/javascript";
	} else if (path.endsWith(".png")) {
		dataType = "image/png";
	} else if (path.endsWith(".gif")) {
		dataType = "image/gif";
	} else if (path.endsWith(".jpg")) {
		dataType = "image/jpeg";
	} else if (path.endsWith(".ico")) {
		dataType = "image/x-icon";
	} else if (path.endsWith(".xml")) {
		dataType = "text/xml";
	} else if (path.endsWith(".pdf")) {
		dataType = "application/pdf";
	} else if (path.endsWith(".zip")) {
		dataType = "application/zip";
	}

	File dataFile = SD.open(path.c_str());
	DBG_OUTPUT_PORT.println("Sending " + path + "...");
	if (dataFile.isDirectory()) {
		path += "/index.html";
		dataType = "text/html";
		dataFile = SD.open(path.c_str());
	}

	if (!dataFile) {
		return false;
	}

	if (server.hasArg("download")) {
		dataType = "application/octet-stream";
	}

	if (server.streamFile(dataFile, dataType) != dataFile.size()) {
		DBG_OUTPUT_PORT.println("Sent less data than expected!");
	}

	dataFile.close();
	return true;
}

void handleFileUpload() {
	if (server.uri() != "/up") {
		return;
	}
	HTTPUpload& upload = server.upload();
	if (upload.status == UPLOAD_FILE_START) {
		String dirname = server.arg("dir");
		if(dirname != "" && !SD.exists((char *)dirname.c_str())) {
			DBG_OUTPUT_PORT.println("Creating directory:" + dirname);
			SD.mkdir((char *)dirname.c_str());
		}
		String path = dirname + "/" + upload.filename;
		uploadFile = SD.open(path.c_str(), FILE_WRITE);
		DBG_OUTPUT_PORT.print("Upload: START, filename: "); DBG_OUTPUT_PORT.println(path);
	} else if (upload.status == UPLOAD_FILE_WRITE) {
		if (uploadFile) {
			uploadFile.write(upload.buf, upload.currentSize);
//		DBG_OUTPUT_PORT.print("Upload: WRITE, Bytes: "); DBG_OUTPUT_PORT.println(upload.currentSize);
		}
	} else if (upload.status == UPLOAD_FILE_END) {
		if (uploadFile) {
			uploadFile.close();
			DBG_OUTPUT_PORT.print("Upload: END, Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
		} else {
			DBG_OUTPUT_PORT.print("Upload: FAIL, "); DBG_OUTPUT_PORT.println(upload.filename);
		}
	}
}

void deleteRecursive(String path) {
	File file = SD.open((char *)path.c_str());
	if (!file.isDirectory()) {
		file.close();
		DBG_OUTPUT_PORT.println("Deleting file " + path);
		SD.remove((char *)path.c_str());
		return;
	}

	file.rewindDirectory();
	while (true) {
		File entry = file.openNextFile();
		if (!entry) {
			break;
		}
		entry.getName(namebuf, sizeof(namebuf));
		String entryPath = path + "/" + namebuf;
		if (entry.isDirectory()) {
			entry.close();
			deleteRecursive(entryPath);
		} else {
			entry.close();
			DBG_OUTPUT_PORT.println("Deleting file entry " + entryPath);
			SD.remove((char *)entryPath.c_str());
		}
		yield();
	}

		DBG_OUTPUT_PORT.println("Deleting directory " + path);
	SD.rmdir((char *)path.c_str());
	file.close();
}

void handleDeleteDir() {
	if (server.args() == 0) {
		return returnFail("BAD ARGS");
	}
	String path = server.arg("dir");
	if (path == "/" || !SD.exists((char *)path.c_str())) {
		returnFail("BAD PATH");
		return;
	}
	DBG_OUTPUT_PORT.println("Deleting directory " + path);
	deleteRecursive(path);
	returnOK();
}

void handleDeleteFile() {
	if (server.args() == 0) {
		return returnFail("BAD ARGS");
	}
	String path = server.arg("file");
	if (path == "/" || !SD.exists((char *)path.c_str())) {
		returnFail("BAD PATH");
		return;
	}
	DBG_OUTPUT_PORT.println("Deleting file " + path);
	SD.remove((char*)path.c_str());
	returnOK();
}

void printDirectory()
{
	if (!server.hasArg("dir")) {
		return returnFail("BAD ARGS");
	}
	String path = server.arg("dir");
//	FatFile dir = SD.open((char *)path.c_str());
//	DBG_OUTPUT_PORT.println("Opened file " + path + bool(dir));

	if (path != "/" && !SD.exists((char *)path.c_str())) {
		return returnFail("BAD PATH: " + path + ", exists:" + SD.exists((char *)path.c_str()));
	}
	File dir = SD.open((char *)path.c_str());
	path = String();
	if (!dir.isDirectory()) {
		dir.close();
		return returnFail("NOT DIR");
	}
	dir.rewindDirectory();
	server.setContentLength(CONTENT_LENGTH_UNKNOWN);
	server.send(200, "text/json", "");
	WiFiClient client = server.client();

	server.sendContent("[");
	for (int cnt = 0; true; ++cnt) {
		File entry = dir.openNextFile();
		if (!entry) {
			break;
		}

		String output;
		if (cnt > 0) {
			output = ',';
		}

		output += "{\"type\":\"";
		output += (entry.isDirectory()) ? "dir" : "file";
		output += "\",\"name\":\"";
		entry.getName(namebuf, sizeof(namebuf));
		output += namebuf;
		output += "\",\"size\":";
		output += entry.fileSize();
		output += "}";
		server.sendContent(output);
		entry.close();
	}
	server.sendContent("]");
	dir.close();
}

void handleNotFound() {
	String message;
	if (hasSD) {
		if(loadFromSdCard(server.uri())) {
			return;
		}
	} else {
		message += "SDCARD Not Detected\n\n";
	}
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	message += "\nFirst arg: ";
	message += server.arg(0);
	message += "\n";
	for (uint8_t i = 0; i < server.args(); i++) {
		message += " NAME:" + server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
	DBG_OUTPUT_PORT.print(message);
}

void setup(void) {
	DBG_OUTPUT_PORT.begin(115200);
	DBG_OUTPUT_PORT.setDebugOutput(true);
	DBG_OUTPUT_PORT.print("\n");
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	DBG_OUTPUT_PORT.print("Connecting to ");
	DBG_OUTPUT_PORT.println(ssid);

	// Wait for connection
	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < 20) {//wait 10 seconds
		delay(500);
	}
	if (i == 21) {
		DBG_OUTPUT_PORT.print("Could not connect to");
		DBG_OUTPUT_PORT.println(ssid);
		while (1) {
			delay(500);
		}
	}
	DBG_OUTPUT_PORT.print("Connected! IP address: ");
	DBG_OUTPUT_PORT.println(WiFi.localIP());

	if (MDNS.begin(host)) {
		MDNS.addService("http", "tcp", 80);
		DBG_OUTPUT_PORT.println("MDNS responder started");
		DBG_OUTPUT_PORT.print("You can now connect to http://");
		DBG_OUTPUT_PORT.print(host);
		DBG_OUTPUT_PORT.println(".local");
	}


	server.on("/ls", HTTP_GET, printDirectory);
	server.on("/rmdir", HTTP_GET, handleDeleteDir);
	server.on("/rm", HTTP_GET, handleDeleteFile);
	server.on("/up", HTTP_POST, []() { returnOK(); }, handleFileUpload);
	server.onNotFound(handleNotFound);

	server.begin();
	DBG_OUTPUT_PORT.println("HTTP server started");

	if (SD.begin(SD_CHIP_SELECT)) {
		DBG_OUTPUT_PORT.println("SD Card initialized.");
		hasSD = true;
	}
}

void loop(void) {
	server.handleClient();
	MDNS.update();
}
