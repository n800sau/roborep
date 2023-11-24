#include "ESP8266WiFi.h"
#include <Servo.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <wifi_utils.h>
#include <misc_utils.h>
#include <ESP_EEPROM.h>

#include "config.h"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define LED_PIN 12

#define T_SRV_PIN D2
#define H_SRV_PIN D3
#define A_SRV_PIN D4

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
#define SLEEP_SECS 10

#define TIMEZONE "Australia/Sydney"

#define FAIL_COUNT_MAX 10

#define PERMDATA_CODE 0xF3E0

struct _PERMDATA {
	uint16_t code;
	int fail_count;
} permdata = {.code=PERMDATA_CODE, .fail_count=0};

void goto_sleep()
{
	EEPROM.put(0, permdata);
	if(!EEPROM.commit()) {
		Serial.println("Commit to EEPROM failed");
	}
	Serial.println("Goto sleep");
	goto_deepsleep(SLEEP_SECS);
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
	if(old_t != t) {
		old_t = t;
		pos = map(t, T_MIN, T_MAX, SRV_MIN, SRV_MAX);
		t_servo.attach(T_SRV_PIN);
		Serial.printf("t pos: %d\n", pos);
		t_servo.write(pos);
		delay(500);
		t_servo.detach();
	}
	if(old_h != h) {
		old_h = h;
		pos = map(h, H_MIN, H_MAX, SRV_MIN, SRV_MAX);
		h_servo.attach(H_SRV_PIN);
		Serial.printf("h pos: %d\n", pos);
		h_servo.write(pos);
		delay(500);
		h_servo.detach();
	}
	if(old_a != a) {
		old_a = a;
		pos = map(a, A_MIN, A_MAX, SRV_MIN, SRV_MAX);
		a_servo.attach(A_SRV_PIN);
		Serial.printf("a pos: %d\n", pos);
		a_servo.write(pos);
		delay(500);
		a_servo.detach();
	}
}

void check_failure()
{
	if(permdata.fail_count > FAIL_COUNT_MAX) {
		t = h = a = 0;
		apply2servo();
	}
}

void setup()
{
	Serial.begin(115200);
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	EEPROM.begin(sizeof(permdata));
	_PERMDATA d;
	EEPROM.get(0, d);
	if(d.code == PERMDATA_CODE) {
		permdata = d;
	}
	WiFi.mode(WIFI_STA);
	if(!connectWifi(ssid, password)) {
		permdata.fail_count++;
		check_failure();
		// can not connect. just goto sleep
		goto_sleep();
	}
	configTime(TIMEZONE, "pool.ntp.org", "time.nist.gov", "time.windows.com");
	digitalWrite(LED_PIN, LOW);
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
					permdata.fail_count = 0;
					apply2servo();
				}
			}
		} else {
			Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
			permdata.fail_count++;
		}
		http.end();
	} else {
		Serial.printf("[HTTP} Unable to connect\n");
		permdata.fail_count++;
	}
	check_failure();
	goto_sleep();
}

