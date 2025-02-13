#include <Arduino.h>

#ifdef ESP32
		#include <WiFi.h>
#else
		#include <ESP8266WiFi.h>
#endif
#include <SD.h>
#include "AudioFileSourceHTTPStream.h"
#include "AudioFileSourceBuffer.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"

#include "config.h"

// To run, set your ESP8266 build to 160MHz, update the SSID info, and upload.

// Enter your WiFi setup here:
const char *SSID = WIFI_SSID;
const char *PASSWORD = WIFI_PASSWORD;

// Randomly picked URL
//const char *URL="http://streaming.shoutcast.com/80sPlanet?lang=en-US";
const char *URL="http://p24t.local/~n800s/aud/pno-cs.mp3";

AudioGeneratorMP3 *mp3;
AudioFileSourceHTTPStream *file;
AudioFileSourceBuffer *buff;
AudioOutputI2SNoDAC *out;

// Called when a metadata event occurs (i.e. an ID3 tag, an ICY block, etc.
void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string)
{
	const char *ptr = reinterpret_cast<const char *>(cbData);
	(void) isUnicode; // Punt this ball for now
	// Note that the type and string may be in PROGMEM, so copy them to RAM for printf
	char s1[32], s2[64];
	strncpy_P(s1, type, sizeof(s1));
	s1[sizeof(s1)-1]=0;
	strncpy_P(s2, string, sizeof(s2));
	s2[sizeof(s2)-1]=0;
	Serial.printf("METADATA(%s) '%s' = '%s'\n", ptr, s1, s2);
	Serial.flush();
}

// Called when there's a warning or error (like a buffer underflow or decode hiccup)
void StatusCallback(void *cbData, int code, const char *string)
{
	const char *ptr = reinterpret_cast<const char *>(cbData);
	// Note that the string may be in PROGMEM, so copy it to RAM for printf
	char s1[64];
	strncpy_P(s1, string, sizeof(s1));
	s1[sizeof(s1)-1]=0;
	Serial.printf("STATUS(%s) '%d' = '%s'\n", ptr, code, s1);
	Serial.flush();
}


void setup()
{
	Serial.begin(115200);
	delay(1000);
	Serial.println("Connecting to WiFi");

	WiFi.disconnect();
	WiFi.softAPdisconnect(true);
	WiFi.mode(WIFI_STA);
	
	WiFi.begin(SSID, PASSWORD);

	// Try forever
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(1000);
	}
	Serial.println("\nConnected");

	audioLogger = &Serial;
	file = new AudioFileSourceHTTPStream(URL);
	file->RegisterMetadataCB(MDCallback, (void*)"HTTP");
	file->RegisterStatusCB(StatusCallback, (void*)"HTTP");
	buff = new AudioFileSourceBuffer(file, 2048);
	buff->RegisterStatusCB(StatusCallback, (void*)"buffer");
	out = new AudioOutputI2SNoDAC();
	mp3 = new AudioGeneratorMP3();
	mp3->RegisterStatusCB(StatusCallback, (void*)"mp3");
	mp3->begin(buff, out);
}

bool finished = false;

void loop()
{
	static int lastms = 0;

	if (mp3->isRunning()) {
		if (millis()-lastms > 1000) {
			lastms = millis();
			Serial.printf("Running for %d ms...\n", lastms);
			Serial.flush();
		 }
		if (!mp3->loop()) mp3->stop();
	} else if(!finished) {
		finished = true;
		Serial.printf("MP3 done\n");
		delay(1000);
	}
}

