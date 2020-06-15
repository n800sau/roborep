#include "WiFi.h"
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include "AsyncUDP.h"
#include <ArduinoJson.h>
#include <blinker.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <SPI.h>

#include "config.h"
const char * ssid = WIFI_SSID;
const char * password = WIFI_PASSWORD;

#define TFT_CS   13
#define TFT_RST  26 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC   12 //A0
#define TFT_MOSI 14  // SDA Data out
#define TFT_SCLK 27  // SCK Clock out

#define LED_PIN 11
Blinker blinker;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
GFXcanvas1 dbuf(180, 120);

#define HOSTNAME "esp32display"

WiFiMulti wifiMulti;
AsyncUDP udp;

//StaticJsonDocument<400> doc;
DynamicJsonDocument doc(400);

time_t read_ts = 0, display_ts = 0;
float t, h, co2;

typedef struct _ap {
	String ssid;
	String password;
} ap_t;

ap_t aplist[] = {
	{WIFI_SSID, WIFI_PASSWORD}
#ifdef WIFI_SSID_1
	,{WIFI_SSID_1, WIFI_PASSWORD_1}
#endif
#ifdef WIFI_SSID_2
	,{WIFI_SSID_2, WIFI_PASSWORD_2}
#endif
};

const int ap_count = sizeof(aplist)/sizeof(ap_t);

String find_better_ap(int thresh=20)
{
	String rs;
	int max_power = WiFi.RSSI();
	int n = WiFi.scanNetworks();
	if(n > 0) {
		Serial.print(n);
		Serial.println(" networks found");
		for (int i = 0; i < n; ++i) {
			// Print SSID and RSSI for each network found
			Serial.print(i + 1);
			Serial.print(": ");
			Serial.print(WiFi.SSID(i));
			Serial.print(" (");
			Serial.print(WiFi.RSSI(i));
			Serial.println(")");
			for(int j=0; j<ap_count; j++) {
				if(WiFi.SSID(i) == aplist[j].ssid) {
					if(WiFi.RSSI(i) - thresh > max_power) {
						max_power = WiFi.RSSI(i);
						rs = WiFi.SSID(i);
					}
				}
			}
		}
	}
	return rs;
}

bool is_wifi_connected()
{
	return wifiMulti.run() == WL_CONNECTED;
}

void connectWifi()
{
	Serial.println("Connecting to WiFi");
	wifiMulti.run(10000);
	if (is_wifi_connected())
	{
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(ssid);
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
		Serial.println("");
		Serial.println("Connection failed.");
	}
}

void connectUDP()
{
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
			String data = packet.readString();
			Serial.print(", Data: ");
			Serial.print(data);
//			Serial.write(packet.data(), packet.length());
			Serial.println();
			//reply to the client
			packet.printf("Got %u bytes of data", packet.length());
			DeserializationError error = deserializeJson(doc, data);
			// Test if parsing succeeded.
			if (error) {
				Serial.print("deserializeMsgPack() failed: ");
				Serial.println(error.c_str());
			} else {
				if(doc["sensor_id"] == String("MQ135")) {
					Serial.print("Sensor:");
					Serial.print((const char *)doc["sensor_id"]);
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

void setup()
{
	Serial.begin(115200);
	Serial.print("setup starts at ");
	Serial.println(millis()/1000);

	blinker.begin(LED_PIN);
	digitalWrite(LED_PIN, HIGH);

	tft.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
	tft.fillScreen(ST77XX_BLACK);
	tft.setFont(&FreeMonoBoldOblique12pt7b);
	tft.setTextSize(0.5);
	tft.setCursor(0, 30);
	tft.setTextColor(ST77XX_WHITE);
	Serial.print(F("TFT initialized at "));
	Serial.println(millis()/1000);

	WiFi.mode(WIFI_STA);
	for(int i=0; i<ap_count; i++) {
		wifiMulti.addAP(aplist[i].ssid.c_str(), aplist[i].password.c_str());
	}

}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(is_wifi_connected())
	{
//		server.handleClient();
		if(udp.connected()) {
			if(read_ts != display_ts) {
				display_ts = read_ts;
				tft.fillScreen(ST77XX_BLACK);
				tft.setTextSize(1);
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
				String better_ssid = find_better_ap(-100);
				if(better_ssid.length() > 0) {
					Serial.print("Better found:");
					Serial.print(better_ssid);
					Serial.print(", connecting..");
					WiFi.disconnect();
				}
			}
		} else {
			connectUDP();
		}
	} else {
		blinker.stop();
		digitalWrite(LED_PIN, HIGH);
		connectWifi();
		if(is_wifi_connected()) {
			WiFi.setHostname(HOSTNAME);
			digitalWrite(LED_PIN, LOW);
			Serial.println("Config time");
			configTime(0, 0, "pool.ntp.org", "time.nist.gov");
			if(MDNS.begin(HOSTNAME)) {
				Serial.println("MDNS responder " HOSTNAME " started");
				// Add service to MDNS-SD
				MDNS.addService("http", "tcp", 80);
			} else {
				Serial.println("Error setting up MDNS responder!");
			}
		}
	}
	delay(10);
}
