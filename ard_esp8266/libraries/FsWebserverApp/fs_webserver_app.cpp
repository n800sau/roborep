#include "fs_webserver_app.h"

#define PIN_HEAT 4
#define PIN_COOL 5
#define PIN_T1 A0
#define PIN_T2 A1

#define DBG_OUTPUT_PORT Serial

#include "config.h"

FSWifiWebServerApp::FSWifiWebServerApp(String host, String username, String password, int port):
	host(host), username(username), password(password), webserver(port)
{
}


//format bytes
String FSWifiWebServerApp::formatBytes(size_t bytes)
{
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

String FSWifiWebServerApp::getContentType(String filename)
{
	if (webserver.hasArg("download")) {
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

bool FSWifiWebServerApp::handleFileRead(String path)
{
	if(is_allowed()) {
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
			webserver.streamFile(file, contentType);
			file.close();
			return true;
		}
	}
	return false;
}

void FSWifiWebServerApp::handleFileUpload()
{
	if(is_allowed()) {
		if (webserver.uri() != "/edit") {
			return;
		}
		HTTPUpload& upload = webserver.upload();
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
}

void FSWifiWebServerApp::handleFileDelete()
{
	if(is_allowed()) {
		if (webserver.args() == 0) {
			return webserver.send(500, "text/plain", "BAD ARGS");
		}
		String path = webserver.arg(0);
		DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
		if (path == "/") {
			return webserver.send(500, "text/plain", "BAD PATH");
		}
		if (!SPIFFS.exists(path)) {
			return webserver.send(404, "text/plain", "FileNotFound");
		}
		SPIFFS.remove(path);
		webserver.send(200, "text/plain", "");
		path = String();
	}
}

void FSWifiWebServerApp::handleFileCreate()
{
	if(is_allowed()) {
		if (webserver.args() == 0) {
			return webserver.send(500, "text/plain", "BAD ARGS");
		}
		String path = webserver.arg(0);
		DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
		if (path == "/") {
			return webserver.send(500, "text/plain", "BAD PATH");
		}
		if (SPIFFS.exists(path)) {
			return webserver.send(500, "text/plain", "FILE EXISTS");
		}
		File file = SPIFFS.open(path, "w");
		if (file) {
			file.close();
		} else {
			return webserver.send(500, "text/plain", "CREATE FAILED");
		}
		webserver.send(200, "text/plain", "");
		path = String();
	}
}

void FSWifiWebServerApp::handleFileList()
{
	if(is_allowed()) {
		if (!webserver.hasArg("dir")) {
			webserver.send(500, "text/plain", "BAD ARGS");
			return;
		}

		String path = webserver.arg("dir");
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
		webserver.send(200, "text/json", output);
	}
}

bool FSWifiWebServerApp::is_allowed()
{
	bool rs = webserver.authenticate(username.c_str(), password.c_str());
	if(!rs) {
		webserver.requestAuthentication();
	}
	return rs;
}

void FSWifiWebServerApp::setup()
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


	WiFiManager wifiManager;

	//reset saved settings
	//wifiManager.resetSettings();

	//set custom ip for portal
	//wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

	//fetches ssid and pass from eeprom and tries to connect
	//if it does not connect it starts an access point with the specified name
	if (!wifiManager.autoConnect(host.c_str())) {
		Serial.println("failed to connect and hit timeout");
		delay(3000);
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(5000);
	}

//	while (WiFi.status() != WL_CONNECTED) {
//		delay(500);
//		DBG_OUTPUT_PORT.print(".");
//	}
	DBG_OUTPUT_PORT.println();
	DBG_OUTPUT_PORT.print("Connected! IP address: ");
	DBG_OUTPUT_PORT.println(WiFi.localIP());

	MDNS.begin(host.c_str());
	DBG_OUTPUT_PORT.print("Open http://");
	DBG_OUTPUT_PORT.print(host);
	DBG_OUTPUT_PORT.println(".local/edit to see the file browser");


	//SERVER INIT
	webserver.on("/", [this]() {
		if(is_allowed()) {
			webserver.sendHeader("Location", "/index.html");
			webserver.sendHeader("Cache-Control", "no-cache");
			webserver.send(302);
		}
	});
	//list directory
	webserver.on("/list", HTTP_GET, std::bind(&FSWifiWebServerApp::handleFileList, this));
	//load editor
	webserver.on("/edit", HTTP_GET, std::bind(&FSWifiWebServerApp::handleFileList, this));
//	webserver.on("/edit", HTTP_GET, [this]() {
//		if (!handleFileRead("/edit.htm")) {
//			webserver.send(404, "text/plain", "FileNotFound");
//		}
//	});
	//create file
	webserver.on("/edit", HTTP_PUT, std::bind(&FSWifiWebServerApp::handleFileCreate, this));
	//delete file
	webserver.on("/edit", HTTP_DELETE, std::bind(&FSWifiWebServerApp::handleFileDelete, this));
	//first callback is called after the request has ended with all parsed arguments
	//second callback handles file uploads at that location
	webserver.on("/edit", HTTP_POST, [this]() {
		if(is_allowed()) {
			webserver.send(200, "text/plain", "");
		}
	}, std::bind(&FSWifiWebServerApp::handleFileUpload, this));

	//called when the url is not defined here
	//use it to load content from SPIFFS
	webserver.onNotFound([this]() {
		if (!handleFileRead(webserver.uri())) {
			webserver.send(404, "text/plain", "FileNotFound");
		}
	});

	//get heap status, analog input value and all GPIO statuses in one json call
	webserver.on("/all", HTTP_GET, [this]() {
		String json = "{";
		json += "\"heap\":" + String(ESP.getFreeHeap());
		json += ", \"analog\":" + String(analogRead(A0));
		json += ", \"gpio\":" + String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
		json += ", \"HTTP_X_FORWARDED_PREFIX\":" + webserver.header("HTTP_X_FORWARDED_PREFIX");

		json += "}";
		webserver.send(200, "text/json", json);
		json = String();
	});

	webserver.begin();
	DBG_OUTPUT_PORT.println("HTTP webserver started");

}

void FSWifiWebServerApp::update_process()
{
}

void FSWifiWebServerApp::loop()
{
	update_process();
	webserver.handleClient();
}
