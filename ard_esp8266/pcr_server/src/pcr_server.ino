// ntc-mf52at b 3950 5% 100k
// http://esp8266tutorials.blogspot.com/2016/09/esp8266-ntc-temperature-thermistor.html

// 3435 10k ntc mf5b 

#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include <SPIFFSEditor.h>
#include <Ticker.h>
#include <ESP8266AVRISP.h>
#include <SPI.h>

#include "config.h"

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const uint16_t AVR_PROGRAMMER_PORT = 328;
const uint8_t AVR_RESET_PIN = 5;
ESP8266AVRISP avrprog(AVR_PROGRAMMER_PORT, AVR_RESET_PIN);

const byte R_CURR_TEMP = 10;

Ticker flipper;
bool heating;
bool cooling;

bool running = false;
unsigned long start_millis;
int secs_remaining;
int group_index;
int repeat;
int stage_index;
int stage = 0;

double temp2set = -10000;
double curr_temp = 0;
const double max_cold_temp = 30;

const char *settings_path = "/settings.json";

void readStageAndTemp(int &stage, double &temp)
{
	stage = 0;
	temp = 0;
	if(SPIFFS.exists(settings_path)) {
		File file = SPIFFS.open(settings_path, "r");
		// Allocate the memory pool on the stack.
		// Don't forget to change the capacity to match your JSON document.
		// Use arduinojson.org/assistant to compute the capacity.
		StaticJsonBuffer<1024> jsonBuffer;
		// Parse the root object
		JsonObject &root = jsonBuffer.parseObject(file);
		if(root.success()) {
			long secs = (millis() - start_millis)/1000;
			for(unsigned int i=0; i<root["settings"].size(); i++) {
Serial.print("group:");
Serial.println(i);
				JsonObject &group = root["settings"][i];
				for(unsigned int j=0; j<group["repeat"]; j++) {
Serial.print("repeat:");
Serial.println(j);
					for(unsigned int k=0; k<group["items"].size(); k++) {
Serial.print("item:");
Serial.println(k);
//Serial.print("secs:");
//Serial.println(secs);
						secs -= (int)group["items"][k]["duration"];
Serial.print("duration:");
Serial.println((int)group["items"][k]["duration"]);
Serial.print("secs remaining:");
Serial.println(secs);
						if(secs < 0) {
							group_index = i;
							repeat = j;
							stage_index = k;
							stage = k + 1;
							temp = group["items"][k]["temp"];
							secs_remaining = -secs;
							break;
						}
					}
					if(stage>0) {
						break;
					}
				}
				if(stage>0) {
					break;
				}
			}
			if(!stage) {
				Serial.println("Finished");
				stop_running();
			}
		} else {
			Serial.println(F("Failed to read settings"));
		}
	}
}

void stop_running()
{
	running = false;
	temp2set = -10000;
}

AsyncCallbackJsonWebHandler* settings_handler = new AsyncCallbackJsonWebHandler("/settings.json",[](AsyncWebServerRequest *request, JsonVariant &json) {
	if(request->method() == HTTP_GET) {
		if(SPIFFS.exists(settings_path)) {
			File file = SPIFFS.open(settings_path, "r");
			request->send(file, "text/json");
//		request->streamFile(file, "text/json");
			file.close();
		} else {
			request->send(200, "text/json", "[]");
		}
	} else if(request->method() == HTTP_POST) {
		JsonObject& jsonObj = json.as<JsonObject>();
		File file = SPIFFS.open(settings_path, "w");
		jsonObj.printTo(file);
		file.close();
		jsonObj.printTo(Serial);
		request->send(200, "text/json", "{\"Result\":\"OK\"}");
	}
});


AsyncCallbackJsonWebHandler* temp_handler = new AsyncCallbackJsonWebHandler("/temp/set",[](AsyncWebServerRequest *request, JsonVariant &json) {
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonObj = json.as<JsonObject>();
	temp2set = jsonObj["temp"];
	Serial.print("temp2set:");
	Serial.println(temp2set);
	JsonObject& root = jsonBuffer.createObject();
	root["temp2set"] = (int)temp2set;
	String output;
	root.printTo(output);
	request->send(200, "text/json", output);
});

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
	if(type == WS_EVT_CONNECT){
		Serial.printf("ws[%s][%d] connect\n", server->url(), client->id());
		client->printf("{\"msg\":\"Hello Client %d\"}", client->id());
		client->ping();
	} else if(type == WS_EVT_DISCONNECT){
		Serial.printf("ws[%s][%d] disconnect\n", server->url(), client->id());
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
const char * hostName = "pcr-server";
const char* http_username = "admin";
const char* http_password = "parol";

void update_temp()
{
	curr_temp = spi_read(R_CURR_TEMP)/10.;
}

uint32_t spi_read(byte reg)
{
	uint32_t rs=0;
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV8);
	SPI.transfer(reg);
	SPI.transferBytes((uint8_t*)&rs, (uint8_t*)&rs, sizeof(rs));
	SPI.end();
	Serial.print(reg);
	Serial.print(" = ");
	Serial.println(rs);
	return rs;
}

void flip()
{
	StaticJsonBuffer<300> jsonBuffer;
	update_heater();
	JsonObject& root = jsonBuffer.createObject();
	root["temp"] = curr_temp;
	if(temp2set > -1000) {
		root["temp2set"] = (int)temp2set;
	}
	root["heating"] = heating;
	root["cooling"] = cooling;
	if(running) {
		root["running_time"] = millis() - start_millis;
		root["secs_remaining"] = secs_remaining;
		root["group_index"] = group_index;
		root["repeat"] = repeat;
		root["stage_index"] = stage_index;
		root["stage"] = stage;
	}
	String output;
	root.printTo(output);

	root.printTo(Serial);
	Serial.println();

	ws.printfAll(output.c_str());
}

void setup()
{
	running = false;
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
	MDNS.addService("avrisp", "tcp", AVR_PROGRAMMER_PORT);

	SPIFFS.begin();
//	SPIFFS.format();

	ws.onEvent(onWsEvent);
	server.addHandler(&ws);


	server.addHandler(temp_handler);
	server.addHandler(settings_handler);

	server.addHandler(new SPIFFSEditor(http_username,http_password));

	server.on("/run", HTTP_GET, [](AsyncWebServerRequest *request){
		running = true;
		start_millis = millis();
		request->send(200, "text/plain", String(ESP.getFreeHeap()));
	});

	server.on("/abort", HTTP_GET, [](AsyncWebServerRequest *request){
		stop_running();
		request->send(200, "text/plain", String(ESP.getFreeHeap()));
	});

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

	update_temp();

	// listen for avrdudes
	avrprog.begin();

	flipper.attach(2, flip);
}

void update_heater()
{
	update_temp();

	// find current stage
	if(running) {
		readStageAndTemp(stage, temp2set);
	}

	// test if temp does not change for some time then stop heating and cooling?

	heating = false;
	cooling = false;

	double dt = temp2set - curr_temp;
	if(dt > 0.5) {
		// heat
		heating = true;
	} else if(dt < -0.5) {
		// cool
		cooling = true;
	}

	if(heating) {
		Serial.println("heating...");
	}
	if(cooling) {
		Serial.println("cooling...");
	}
}

void loop()
{
	static AVRISPState_t last_state = AVRISP_STATE_IDLE;
	AVRISPState_t new_state = avrprog.update();
	if (last_state != new_state) {
		switch (new_state) {
			case AVRISP_STATE_IDLE: {
					Serial.printf("[AVRISP] now idle\r\n");
					// Use the SPI bus for other purposes
					break;
				}
			case AVRISP_STATE_PENDING: {
					Serial.printf("[AVRISP] connection pending\r\n");
					// Clean up your other purposes and prepare for programming mode
					break;
				}
			case AVRISP_STATE_ACTIVE: {
					Serial.printf("[AVRISP] programming mode\r\n");
					// Stand by for completion
					break;
				}
		}
		last_state = new_state;
	}
	// Serve the client
	if (last_state != AVRISP_STATE_IDLE) {
		avrprog.serve();
	} else {
		// allow to run
	}
}
