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
#include <PID_v1.h>

#include "config.h"

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define TEMP_PIN A0
#define COOL_PIN 5
#define HEAT_PIN 4

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
const double max_cold_temp = 30;
double v, r, temp;
double Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID heatPID(&temp, &Output, &temp2set, Kp, Ki, Kd, DIRECT);

unsigned long WindowSize = 5000;
unsigned long windowStartTime;

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
const char * hostName = "heat-server";
const char* http_username = "admin";
const char* http_password = "parol";

int tempAnalogRead()
{
	int val = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(TEMP_PIN);
		delay(1);
	}

	val = val / 20;
	return val;
}

double Vcc = 3.3;
const double T_0 = 273.15;
const double T_25 = T_0 + 25;
// 100k
//const double beta = 3950;
//const double R_25 = 100000; // 100k ohm
//const unsigned int Rs = 270000;


// 10k
const double beta = 3435;
const double R_25 = 10000; // 10k ohm
//const unsigned int Rs = 32600;
const unsigned int Rs = 33000;


double thermister(double r)
{
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
}

void update_temp()
{
//	Serial.print("A0:");
//	Serial.println(analogRead(TEMP_PIN));
	v = tempAnalogRead()/1024.;
	r = v/((3.3-v)/Rs);
	temp = thermister(r);
}

void flip()
{
	StaticJsonBuffer<300> jsonBuffer;
	update_heater();
	JsonObject& root = jsonBuffer.createObject();
	root["v"] = v;
	root["r"] = r;
	root["temp"] = temp;
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
	pinMode(HEAT_PIN, OUTPUT);
	pinMode(COOL_PIN, OUTPUT);
	digitalWrite(HEAT_PIN, LOW);
	digitalWrite(COOL_PIN, LOW);
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
	windowStartTime = millis();
	//tell the PID to range between 0 and the full window size
	heatPID.SetOutputLimits(0, WindowSize);
	//turn the PID on
	heatPID.SetMode(AUTOMATIC);

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

/*
	if(temp2set > -1000) {
		heatPID.Compute();
		if(millis() - windowStartTime > WindowSize) {
			//time to shift the Relay Window
			windowStartTime += WindowSize;
		}
		heating = Output < millis() - windowStartTime;
		cooling = (temp < max_cold_temp) ? false : !heating;
	}
*/

	double dt = temp2set-temp;
	if(dt > 0.5) {
		// heat
		heating = true;
	} else if(dt < -0.5) {
		// cool
		cooling = true;
	}

	digitalWrite(HEAT_PIN, heating ? HIGH : LOW);
	digitalWrite(COOL_PIN, cooling ? HIGH : LOW);

	if(heating) {
		Serial.println("heating...");
	}
	if(cooling) {
		Serial.println("cooling...");
	}
}

void loop()
{
}
