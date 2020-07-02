#include "ESP8266WiFi.h"
#include <Servo.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <wifi_utils.h>
#include <misc_utils.h>

#include "config.h"
const char *ssid = WIFI_SSID_1;
const char *password = WIFI_PASSWORD_1;

#define T_SRV_PIN 2
#define H_SRV_PIN 4
#define A_SRV_PIN 5

Servo t_servo;
Servo h_servo;
Servo a_servo;
// twelve servo objects can be created on most boards

float old_t=0, old_h=0, old_a=0;
float t=0, h=0, a=0;

#define T_MIN 0
#define T_MAX 40

#define H_MIN 40
#define H_MAX 100

#define A_MIN 0
#define A_MAX 10

#define SRV_MIN 0
#define SRV_MAX 180

// time to sleep
#define SLEEP_SECS 300

void setup()
{
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	if(!connectWifi(ssid, password)) {
		// can not connect. just sleep
		goto_deepsleep(SLEEP_SECS);
	}
}

bool process_json(String sensor_data)
{
	bool rs = false;
	DynamicJsonDocument doc(400);
	if(sensor_data.length()) {
		Serial.println("Process json");
		DeserializationError error = deserializeJson(doc, sensor_data);
		// Test if parsing succeeded.
		if (error) {
			Serial.print("deserializeMsgPack() failed: ");
			Serial.println(error.c_str());
		} else {
			if(doc["sensor_id"] == String("MQ135")) {
				t = doc["temperature"];
				h = doc["humidity"];
				a = doc["air"];
				print_ts_prefix();
				Serial.print(": Sensor:");
				Serial.println((const char *)doc["sensor_id"]);
				rs = true;
			}
		}
		sensor_data = "";
	}
	return rs;
}

void apply2servo()
{
	int pos;
	bool changed = false;
	if(old_t != t) {
		old_t = t;
		pos = map(t, T_MIN, T_MAX, SRV_MIN, SRV_MAX);
		t_servo.attach(T_SRV_PIN);
		t_servo.write(pos);
		t_servo.detach();
		changed = true;
	}
	if(old_h != h) {
		old_h = h;
		pos = map(t, H_MIN, H_MAX, SRV_MIN, SRV_MAX);
		h_servo.attach(H_SRV_PIN);
		h_servo.write(pos);
		h_servo.detach();
		changed = true;
	}
	if(old_a != a) {
		old_a = a;
		pos = map(t, A_MIN, A_MAX, SRV_MIN, SRV_MAX);
		a_servo.attach(A_SRV_PIN);
		a_servo.write(pos);
		a_servo.detach();
		changed = true;
	}
	if(changed) {
		// wait for some time, let servo finish
		delay(15);
	}
}

void loop()
{
	HTTPClient http;
	WiFiClient client;

	if(http.begin(client, "http://mq135.local/data.json")) {
		http.addHeader("Content-Type","text/json");
		// start connection and send HTTP header
		int httpCode = http.GET();
		// httpCode will be negative on error
		if (httpCode > 0) {
			// HTTP header has been send and Server response header has been handled
			Serial.printf("[HTTP] GET... code: %d\n", httpCode);
			// file found at server
			if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
				String payload = http.getString();
				Serial.println(payload);
				if(process_json(payload)) {
					apply2servo();
				}
			}
		} else {
			Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
		}
		http.end();
	} else {
		Serial.printf("[HTTP} Unable to connect\n");
	}
	goto_deepsleep(SLEEP_SECS);
}

