#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include "config.h"

#define DBG_PORT Serial1
#define STM32_PORT Serial

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* host = "esp8266fs";

const int RESET_SLAVE_PIN = D1;

const char* nc_host = "acer.local";
const int nc_port = 10000;


ESP8266WebServer server(80);
//holds the current upload
File fsUploadFile;

double Kp = -1;
double Kd = -1;
double Ki = -1;
double roff = 0;

//format bytes
String formatBytes(size_t bytes){
	if (bytes < 1024){
		return String(bytes)+"B";
	} else if(bytes < (1024 * 1024)){
		return String(bytes/1024.0)+"KB";
	} else if(bytes < (1024 * 1024 * 1024)){
		return String(bytes/1024.0/1024.0)+"MB";
	} else {
		return String(bytes/1024.0/1024.0/1024.0)+"GB";
	}
}

String getContentType(String filename){
	if(server.hasArg("download")) return "application/octet-stream";
	else if(filename.endsWith(".htm")) return "text/html";
	else if(filename.endsWith(".html")) return "text/html";
	else if(filename.endsWith(".css")) return "text/css";
	else if(filename.endsWith(".js")) return "application/javascript";
	else if(filename.endsWith(".png")) return "image/png";
	else if(filename.endsWith(".gif")) return "image/gif";
	else if(filename.endsWith(".jpg")) return "image/jpeg";
	else if(filename.endsWith(".ico")) return "image/x-icon";
	else if(filename.endsWith(".xml")) return "text/xml";
	else if(filename.endsWith(".pdf")) return "application/x-pdf";
	else if(filename.endsWith(".zip")) return "application/x-zip";
	else if(filename.endsWith(".gz")) return "application/x-gzip";
	return "text/plain";
}

bool handleFileRead(String path){
	DBG_PORT.println("handleFileRead: " + path);
	if(path.endsWith("/")) path += "index.htm";
	String contentType = getContentType(path);
	String pathWithGz = path + ".gz";
	if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
		if(SPIFFS.exists(pathWithGz))
			path += ".gz";
		File file = SPIFFS.open(path, "r");
		size_t sent = server.streamFile(file, contentType);
		file.close();
		return true;
	}
	return false;
}

void handleFileUpload(){
	if(server.uri() != "/edit") return;
	HTTPUpload& upload = server.upload();
	if(upload.status == UPLOAD_FILE_START){
		String filename = upload.filename;
		if(!filename.startsWith("/")) filename = "/"+filename;
		DBG_PORT.print("handleFileUpload Name: "); DBG_PORT.println(filename);
		fsUploadFile = SPIFFS.open(filename, "w");
		filename = String();
	} else if(upload.status == UPLOAD_FILE_WRITE){
		//DBG_PORT.print("handleFileUpload Data: "); DBG_PORT.println(upload.currentSize);
		if(fsUploadFile)
			fsUploadFile.write(upload.buf, upload.currentSize);
	} else if(upload.status == UPLOAD_FILE_END){
		if(fsUploadFile)
			fsUploadFile.close();
		DBG_PORT.print("handleFileUpload Size: "); DBG_PORT.println(upload.totalSize);
	}
}

void handleFileDelete(){
	if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
	String path = server.arg(0);
	print2nc("handleFileDelete: " + path + "\n");
	if(path == "/")
		return server.send(500, "text/plain", "BAD PATH");
	if(!SPIFFS.exists(path))
		return server.send(404, "text/plain", "FileNotFound");
	SPIFFS.remove(path);
	server.send(200, "text/plain", "");
	path = String();
}

void handleFileCreate(){
	if(server.args() == 0)
		return server.send(500, "text/plain", "BAD ARGS");
	String path = server.arg(0);
	DBG_PORT.println("handleFileCreate: " + path);
	if(path == "/")
		return server.send(500, "text/plain", "BAD PATH");
	if(SPIFFS.exists(path))
		return server.send(500, "text/plain", "FILE EXISTS");
	File file = SPIFFS.open(path, "w");
	if(file)
		file.close();
	else
		return server.send(500, "text/plain", "CREATE FAILED");
	server.send(200, "text/plain", "");
	path = String();
}

void handleFileList()
{
	if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
	
	String path = server.arg("dir");
	DBG_PORT.println("handleFileList: " + path);
	Dir dir = SPIFFS.openDir(path);
	path = String();

	String output = "[";
	while(dir.next()){
		File entry = dir.openFile("r");
		if (output != "[") output += ',';
		bool isDir = false;
		output += "{\"type\":\"";
		output += (isDir)?"dir":"file";
		output += "\",\"name\":\"";
		output += String(entry.name()).substring(1);
		output += "\"}";
		entry.close();
	}
	
	output += "]";
	server.send(200, "text/json", output);
}

void print2nc(String msg)
{
	DBG_PORT.print(msg);
	WiFiClient client;
	if(client.connect(nc_host, nc_port)) {
//		DBG_PORT.println("Connected to " + String(nc_host) + ":" + String(nc_port));
		// This will send the request to the server
		client.print(msg);
//		DBG_PORT.println("closing connection");
		client.stop();
	} else {
		DBG_PORT.println("Can not connect to " + String(nc_host) + ":" + String(nc_port));
	}
}

String request_stm32(String cmdline)
{
	STM32_PORT.println(cmdline);
	print2nc("sent:" + cmdline + "\n");
//	STM32_PORT.flush();
	STM32_PORT.readStringUntil('{');
	return "{" + STM32_PORT.readStringUntil('\n');
}

void handleCurrentValues()
{
	String output = request_stm32("pid");
	print2nc("stm32 current values:[" + output + "]\n");
	server.send(200, "text/json", output);
}

void handleSetParam()
{
	print_http_args();
	String name = server.arg("name");
	String value = server.arg("value");
	String output = request_stm32("set " + name + " " + value);
	server.send(200, "text/json", output);
}

void print_http_args()
{
	String message;
	for (uint8_t i=0; i<server.args(); i++){
		if(message.length() > 0) {
			message += "; ";
		}
		message += server.argName(i) + ": " + server.arg(i);
	}
	print2nc("GET args:[" + message + "]\n");
}

void reset_slave()
{
	digitalWrite(RESET_SLAVE_PIN, LOW);
	delay(100);
	digitalWrite(RESET_SLAVE_PIN, HIGH);
}

void setup(void)
{
	DBG_PORT.begin(115200);
	DBG_PORT.print("\n");
	DBG_PORT.setDebugOutput(true);

	STM32_PORT.begin(115200);
	STM32_PORT.swap();
	STM32_PORT.setTimeout(2000);

	pinMode(RESET_SLAVE_PIN, OUTPUT);
	reset_slave();

	SPIFFS.begin();
	{
		Dir dir = SPIFFS.openDir("/");
		while (dir.next()) {		
			String fileName = dir.fileName();
			size_t fileSize = dir.fileSize();
			DBG_PORT.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
		}
		DBG_PORT.printf("\n");
	}

	//WIFI INIT
	DBG_PORT.printf("Connecting to %s\n", ssid);
	if (String(WiFi.SSID()) != String(ssid)) {
		WiFi.begin(ssid, password);
	}
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		DBG_PORT.print(".");
	}
	DBG_PORT.println("");
	print2nc("Connected! IP address: ");
	print2nc(WiFi.localIP().toString());
	print2nc("\n");

	MDNS.begin(host);
	print2nc("Open http://");
	print2nc(host);
	print2nc(".local/edit to see the file browser\n");


	//SERVER INIT
	//list directory
	server.on("/list", HTTP_GET, handleFileList);

	//create file
	server.on("/edit", HTTP_PUT, handleFileCreate);
	//delete file
	server.on("/edit", HTTP_DELETE, handleFileDelete);
	//first callback is called after the request has ended with all parsed arguments
	//second callback handles file uploads at that location
	server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);

	server.on("/current_values", HTTP_GET, handleCurrentValues);
	server.on("/set_param", HTTP_GET, handleSetParam);

	//called when the url is not defined here
	//use it to load content from SPIFFS
	server.onNotFound([](){
		if(!handleFileRead(server.uri()))
			server.send(404, "text/plain", "FileNotFound");
	});

	//get heap status, analog input value and all GPIO statuses in one json call
	server.on("/all", HTTP_GET, [](){
		String json = "{";
		json += "\"heap\":"+String(ESP.getFreeHeap());
		json += ", \"analog\":"+String(analogRead(A0));
		json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
		json += "}";
		server.send(200, "text/json", json);
		json = String();
	});
	server.begin();
	print2nc("HTTP server started\n");

}

void loop()
{
	// drain stm32 output
	while(STM32_PORT.available()) {
		print2nc(STM32_PORT.readStringUntil('\n'));
	}
	server.handleClient();
}
