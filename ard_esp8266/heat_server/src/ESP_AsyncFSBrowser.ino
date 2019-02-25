// ntc-mf52at b 3950 5% 100k
// http://esp8266tutorials.blogspot.com/2016/09/esp8266-ntc-temperature-thermistor.html

// for 10k 3.3v
// 150k -v- NTC(10k)
// v should not be bigger than 1v for esp8266
// vcc * Rntc / (r+Rntc) = Vpin
// 3.3 * 100k /(r + 100k) = 1
// 1/3.3/100k = 1/(r + 100k)
// 1/(1/3.3/100k) = r + 100k
// r = 1/(1/3.3/100k) - 100k = 230k

// 3.27 - 0.75
//270/2.52 = x/0.75
//x=80k (27C)
//0.75/x = 3.27/(270+x)
//0.75/3.27 = x/(270+x)
//0.229*(270+x)/x=1
//0.229*(270/x+1) = 1
//v = 3.27/(270+R)*R

//T=30C
//R = 80.29
//v = 3.27/(270+80.29)*80.29 = 0.7495

//T=35
//R=64.87
//v = 3.27/(270+64.87)*64.87 = 0.63345

unsigned int Rs = 270000;
double Vcc = 3.3;

#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <Ticker.h>

#include "config.h"

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define TEMP_PIN A0

Ticker flipper;

void readSettings(AsyncWebServerRequest *request)
{
	String path = "/settings.json";
	if(SPIFFS.exists(path)) {
		File file = SPIFFS.open(path, "r");
		request->send(file, "text/json");
//		request->streamFile(file, "text/json");
		file.close();
	} else {
		request->send(200, "text/json", "[]");
	}
}


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
	if(type == WS_EVT_CONNECT){
		Serial.printf("ws[%s][%d] connect\n", server->url(), client->id());
		client->printf("{\"msg\":\"Hello Client %d\"}", client->id());
		client->ping();
	} else if(type == WS_EVT_DISCONNECT){
		Serial.printf("ws[%s][%d] disconnect: %d\n", server->url(), client->id());
	} else if(type == WS_EVT_ERROR){
		Serial.printf("ws[%s][%d] error(%d): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
	} else if(type == WS_EVT_PONG){
		Serial.printf("ws[%s][%d] pong[%d]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
	} else if(type == WS_EVT_DATA){
		AwsFrameInfo * info = (AwsFrameInfo*)arg;
		String msg = "";
		if(info->final && info->index == 0 && info->len == len){
			//the whole message is in a single frame and we got all of it's data
			Serial.printf("ws[%s][%d] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

			if(info->opcode == WS_TEXT){
				for(size_t i=0; i < info->len; i++) {
					msg += (char) data[i];
				}
			} else {
				char buff[3];
				for(size_t i=0; i < info->len; i++) {
					sprintf(buff, "%02x ", (uint8_t) data[i]);
					msg += buff ;
				}
			}
			Serial.printf("%s\n",msg.c_str());

			if(info->opcode == WS_TEXT)
				client->text("I got your text message");
			else
				client->binary("I got your binary message");
		} else {
			//message is comprised of multiple frames or the frame is split into multiple packets
			if(info->index == 0){
				if(info->num == 0)
					Serial.printf("ws[%s][%d] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
				Serial.printf("ws[%s][%d] frame[%d] start[%llu]\n", server->url(), client->id(), info->num, info->len);
			}

			Serial.printf("ws[%s][%d] frame[%d] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

			if(info->opcode == WS_TEXT){
				for(size_t i=0; i < info->len; i++) {
					msg += (char) data[i];
				}
			} else {
				char buff[3];
				for(size_t i=0; i < info->len; i++) {
					sprintf(buff, "%02x ", (uint8_t) data[i]);
					msg += buff ;
				}
			}
			Serial.printf("%s\n",msg.c_str());

			if((info->index + len) == info->len){
				Serial.printf("ws[%s][%d] frame[%d] end[%llu]\n", server->url(), client->id(), info->num, info->len);
				if(info->final){
					Serial.printf("ws[%s][%d] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
					if(info->message_opcode == WS_TEXT)
						client->text("I got your text message");
					else
						client->binary("I got your binary message");
				}
			}
		}
	}
}


const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char * hostName = "heat-server";
const char* http_username = "admin";
const char* http_password = "parol";

int analogRead()
{
	int val = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(TEMP_PIN);
		delay(1);
	}

	val = val / 20;
	return val;
}

const double T_0 = 273.15;
const double T_25 = T_0 + 25;
const double beta = 3950;
const double R_25 = 100000; // 100k ohm

double thermister(double r)
{
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
}

void flip()
{
	double v = analogRead()/1024.;
	double r = v/((3.3-v)/270000);
	double temp = thermister(r);
	ws.printfAll("{\"v\": %.2f, \"r\": %.2f, \"temp\": %.2f}\n", v, r, temp);
}


void setup(){
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	WiFi.hostname(hostName);
	WiFi.mode(WIFI_AP_STA);
	WiFi.softAP(hostName);
	WiFi.begin(ssid, password);
	if (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.printf("STA: Failed!\n");
		WiFi.disconnect(false);
		delay(1000);
		WiFi.begin(ssid, password);
	}

	MDNS.begin(hostName);
	MDNS.addService("http","tcp",80);

	SPIFFS.begin();
//	SPIFFS.format();

	ws.onEvent(onWsEvent);
	server.addHandler(&ws);

	server.addHandler(new SPIFFSEditor(http_username,http_password));

	server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
		request->send(200, "text/plain", String(ESP.getFreeHeap()));
	});

	server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

	server.onNotFound([](AsyncWebServerRequest *request){
		Serial.printf("NOT_FOUND: ");
		if(request->method() == HTTP_GET)
			Serial.printf("GET");
		else if(request->method() == HTTP_POST)
			Serial.printf("POST");
		else if(request->method() == HTTP_DELETE)
			Serial.printf("DELETE");
		else if(request->method() == HTTP_PUT)
			Serial.printf("PUT");
		else if(request->method() == HTTP_PATCH)
			Serial.printf("PATCH");
		else if(request->method() == HTTP_HEAD)
			Serial.printf("HEAD");
		else if(request->method() == HTTP_OPTIONS)
			Serial.printf("OPTIONS");
		else
			Serial.printf("UNKNOWN");
		Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

		if(request->contentLength()){
			Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
			Serial.printf("_CONTENT_LENGTH: %d\n", request->contentLength());
		}

		int headers = request->headers();
		int i;
		for(i=0;i<headers;i++){
			AsyncWebHeader* h = request->getHeader(i);
			Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
		}

		int params = request->params();
		for(i=0;i<params;i++){
			AsyncWebParameter* p = request->getParam(i);
			if(p->isFile()){
				Serial.printf("_FILE[%s]: %s, size: %d\n", p->name().c_str(), p->value().c_str(), p->size());
			} else if(p->isPost()){
				Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
			} else {
				Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
			}
		}

		request->send(404);
	});
	server.on("/settings.json", HTTP_GET, readSettings);
//	server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
//		if(!index)
//			Serial.printf("UploadStart: %s\n", filename.c_str());
//		Serial.printf("%s", (const char*)data);
//		if(final)
//			Serial.printf("UploadEnd: %s (%d)\n", filename.c_str(), index+len);
//	});
//	server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
//		if(!index)
//			Serial.printf("BodyStart: %d\n", total);
//		Serial.printf("%s", (const char*)data);
//		if(index + len == total)
//			Serial.printf("BodyEnd: %d\n", total);
//	});
	server.begin();
	flipper.attach(2, flip);
}

void loop(){
}
