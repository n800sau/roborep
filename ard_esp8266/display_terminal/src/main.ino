#include <ESP8266WiFi.h>
#include "config.h"
#include <StreamString.h>

#include <ArduinoJson.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <ESPAsyncUDP.h>

const char * ssid = WIFI_SSID;
const char * password = WIFI_PASSWORD;

#define TFT_CS   D2
#define TFT_RST  D4 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC   D3 //A0
#define TFT_MOSI D7  // SDA Data out
#define TFT_SCLK D5  // SCK Clock out

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
GFXcanvas1 dbuf(180, 120);

AsyncUDP udp;

//StaticJsonDocument<400> doc;
DynamicJsonDocument doc(400);

time_t read_ts = 0, display_ts = 0;
float t, h, co2;

void setup()
{
	tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
	tft.fillScreen(ST77XX_BLACK);

	Serial.begin(115200);
	Serial.print("setup starts at ");
	Serial.println(millis()/1000);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	Serial.print(F("TFT initialized at "));
	Serial.println(millis()/1000);

	if (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("WiFi Failed");
		while(1) {
			delay(1000);
		}
	}
	Serial.print("about to listen at ");
	Serial.println(millis()/1000);
	if(udp.listenMulticast(IPAddress(239, 0, 0, 57), 8989)) {
		Serial.print("UDP Listening on IP: ");
		Serial.println(WiFi.localIP());
		udp.onPacket([](AsyncUDPPacket packet) {
			Serial.print("received at ");
			Serial.println(millis()/1000);
			Serial.print("UDP Packet Type: ");
			Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
			Serial.print(", From: ");
			Serial.print(packet.remoteIP());
			Serial.print(":");
			Serial.print(packet.remotePort());
			Serial.print(", To: ");
			Serial.print(packet.localIP());
			Serial.print(":");
			Serial.print(packet.localPort());
			Serial.print(", Length: ");
			Serial.print(packet.length());
			StreamString data;
			data.write(packet.data(), packet.length());
			Serial.print(", Data: ");
			Serial.print(data);
//			Serial.write(packet.data(), packet.length());
			Serial.println();

//			DeserializationError error = deserializeJson(doc, String("{\"sensor_id\":\"MQ135\",\"ts\":1592014194,\"timestamp\":\"2020-06-13 02:09:54 UTC\",\"temperature\":19.70,\"humidity\":79.30,\"raw\":0.47,\"co2\":5.96,\"r0\":21.71}"));
			DeserializationError error = deserializeJson(doc, String(data));
			// Test if parsing succeeded.
			if (error) {
				Serial.print("deserializeMsgPack() failed: ");
				Serial.println(error.c_str());
			} else {
				if(doc["sensor_id"] == String("MQ135")) {
					Serial.print("Sensor:");
					Serial.print((const char*)doc["sensor_id"]);
					Serial.print(" t:");
					Serial.print((float)doc["temperature"]);
					Serial.print(" h:");
					Serial.print((float)doc["humidity"]);
					Serial.print(" co2:");
					Serial.println((float)doc["co2"]);
					t = doc["temperature"];
					h = doc["humidity"];
					co2 = doc["co2"];
					read_ts = time(NULL);
				}
			}
		});
	}
}

void loop()
{
	if(read_ts != display_ts) {
		display_ts = read_ts;
					tft.fillScreen(ST77XX_BLACK);
					tft.setTextSize(2);
					tft.setCursor(0, 30);
					tft.setTextColor(ST77XX_RED);
					tft.print("t:");
					tft.println(t);
					tft.setCursor(0, 60);
					tft.setTextColor(ST77XX_BLUE);
					tft.print("h:");
					tft.println(h);
					tft.setCursor(0, 90);
					tft.setTextColor(ST77XX_GREEN);
					tft.print("co2:");
					tft.println(co2);
	}
	delay(10);
}
