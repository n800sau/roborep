#include "WiFi.h"
#include <ESPmDNS.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <SD.h>
#include <blinker.h>
#include <HTTPClient.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeMonoBoldOblique24pt7b.h>
#include <SPI.h>

#include "httpsrv.hpp"

#include "config.h"
const char * ssid = WIFI_SSID;
const char * password = WIFI_PASSWORD;

#define K1_PIN 27
#define K2_PIN 26
#define K3_PIN 25
#define K4_PIN 33

// set it high to go to sleep
#define GOTOSLEEP_REQUEST_PIN 14

volatile uint8_t key_state = 0;
enum SCREEN_TYPE {VAL_SCREEN, INFO_SCREEN};
SCREEN_TYPE screen_num = VAL_SCREEN;

#define TFT_CS         4
#define TFT_RST        22
#define TFT_DC         21
#define TFT_LED_PIN    32

//#define TFT_MISO      19
//#define TFT_MOSI      23
//#define TFT_SCLK      18


#define SD_SS 5
// default SPI pins
//#define SPI_MISO 19
//#define SPI_MOSI 23
//#define SPI_CLK  18

bool SD_available = false;


// display resolution 160x128 (160/3 = 41)

#define LED_PIN 34
Blinker blinker;
Ticker check_wifi_ticker;
Ticker keyboard_ticker;
Ticker gotosleep_ticker;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas1 dbuf(180, 120);

#define HOSTNAME "esp32display"

AsyncUDP udp;

//StaticJsonDocument<400> doc;
DynamicJsonDocument doc(400);

volatile time_t read_ts = 0, display_ts = 0, refresh_screen_ts = 0;
float t, h, co2;
#define REFRESH_SCREEN_TIMEOUT 1

volatile bool do_check_for_better_wifi = false;

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

bool is_wifi_connected()
{
	return WiFi.status() == WL_CONNECTED;
}

ap_t find_better_ap(int thresh=20)
{
	ap_t rs;
	int max_power = 0;
	String current_ssid;
	if(is_wifi_connected()) {
		max_power = WiFi.RSSI();
		current_ssid = WiFi.SSID();
	} else {
		thresh = 0;
	}
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
				if(WiFi.SSID(i) == aplist[j].ssid && WiFi.SSID(i) != current_ssid) {
					if(max_power == 0 || WiFi.RSSI(i) - thresh > max_power) {
						max_power = WiFi.RSSI(i);
						rs = aplist[j];
					}
				}
			}
		}
	}
	return rs;
}

void connectWifi()
{
	ap_t ap = find_better_ap();
	if(ap.ssid.length()) {
		Serial.print("Connecting to WiFi ");
		Serial.println(ap.ssid);
		WiFi.begin(ap.ssid.c_str(), ap.password.c_str());
		int i = 0;
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.print(".");
			if (i > 10){
				break;
			}
			i++;
		}
	}
	if (is_wifi_connected())
	{
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(WiFi.SSID());
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
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

void reset_gotosleep_timer()
{
	digitalWrite(GOTOSLEEP_REQUEST_PIN, LOW);
	gotosleep_ticker.once(10, []() {
		Serial.println("Go to sleep");
		gotosleep_ticker.once(1, []() {
			digitalWrite(GOTOSLEEP_REQUEST_PIN, HIGH);
		});
	});
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
	Serial.printf("Listing directory: %s\n", dirname);

	File root = fs.open(dirname);
	if(!root){
		Serial.println("Failed to open directory");
		return;
	}
	if(!root.isDirectory()){
		Serial.println("Not a directory");
		return;
	}

	File file = root.openNextFile();
	while(file){
		if(file.isDirectory()){
			Serial.print("  DIR : ");
			Serial.println(file.name());
			if(levels){
				listDir(fs, file.name(), levels -1);
			}
		} else {
			Serial.print("  FILE: ");
			Serial.print(file.name());
			Serial.print("  SIZE: ");
			Serial.println(file.size());
		}
		file = root.openNextFile();
	}
}

void print_SD_info()
{
	uint8_t cardType = SD.cardType();

	switch(cardType) {
		case CARD_NONE:
			Serial.println("No SD card attached");
			break;
		default: {
			Serial.print("SD Card Type: ");
			switch(cardType) {
				case CARD_MMC:
					Serial.println("MMC");
					break;
				case CARD_SD:
					Serial.println("SDSC");
					break;
				case CARD_SDHC:
					Serial.println("SDHC");
					break;
				default:
					Serial.println("UNKNOWN");
					break;
			}
			uint64_t cardSize = SD.cardSize() / (1024 * 1024);
			Serial.printf("SD Card Size: %lluMB\n", cardSize);
			listDir(SD, "/", 0);
			break;
		}
	}

}

void get_data_now()
{
	HTTPClient http;
	reset_gotosleep_timer();
	http.begin("http://mq135.local/data.json");
	http.addHeader("Content-Type","text/json");
	int httpResponceCode = http.GET();
	if (httpResponceCode == 200) {
		String response = http.getString();
		Serial.println(httpResponceCode);
		Serial.println(response);
		if(response.length() > 0) {
			DeserializationError error = deserializeJson(doc, response);
			if (error) {
				Serial.print("deserializeJson() failed: ");
				Serial.println(error.c_str());
			} else {
				const char* retval1 = doc["sensor_id"][0];
				Serial.println("Sensor Id: ");
				Serial.println(retval1);
			}
		}
	} else {
		Serial.print("Direct connection to sensor failed: ");
		Serial.println(httpResponceCode);
	}
	http.end();
}

void setup()
{
	pinMode(TFT_LED_PIN, OUTPUT);
	// turn off tft led
	digitalWrite(TFT_LED_PIN, HIGH);

	Serial.begin(115200);
	Serial.println();
	Serial.print("setup starts at ");
	Serial.println(millis()/1000);

	blinker.begin(LED_PIN);
	digitalWrite(LED_PIN, HIGH);

	pinMode(K1_PIN, INPUT_PULLUP);
	pinMode(K2_PIN, INPUT_PULLUP);
	pinMode(K3_PIN, INPUT_PULLUP);
	pinMode(K4_PIN, INPUT_PULLUP);

	pinMode(GOTOSLEEP_REQUEST_PIN, OUTPUT);
	digitalWrite(GOTOSLEEP_REQUEST_PIN, LOW);

	SD_available = SD.begin(SD_SS);
	if(!SD_available) {
		Serial.println("Card Mount Failed");
	}

	tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
	tft.fillScreen(ST77XX_BLACK);
	tft.setRotation(3);
	tft.setFont(&FreeMonoBoldOblique12pt7b);
	tft.setTextSize(1);
	tft.setTextColor(ST77XX_WHITE);
	tft.setCursor(0, 30);
	tft.print("Hello there");
	Serial.print(F("TFT initialized at "));
	Serial.println(millis()/1000);
	// turn on tft led
	digitalWrite(TFT_LED_PIN, LOW);

				int16_t xp, yp;
				uint16_t w, h;
				tft.getTextBounds("+88", 0, 0, &xp, &yp, &w, &h);
				Serial.print("bounds of text 12:");
				Serial.print(xp);
				Serial.print(",");
				Serial.print(yp);
				Serial.print(",");
				Serial.print(w);
				Serial.print("x");
				Serial.println(h);
				tft.setFont(&FreeMonoBoldOblique24pt7b);
				tft.getTextBounds("+88", 0, 0, &xp, &yp, &w, &h);
				Serial.print("bounds of text 24:");
				Serial.print(xp);
				Serial.print(",");
				Serial.print(yp);
				Serial.print(",");
				Serial.print(w);
				Serial.print("x");
				Serial.println(h);

	WiFi.mode(WIFI_STA);
	check_wifi_ticker.attach(30, [](){ do_check_for_better_wifi=true; });
	keyboard_ticker.attach(1, []() {
		key_state = 0;
		key_state |= uint8_t(digitalRead(K1_PIN) == HIGH);
		key_state |= uint8_t(digitalRead(K2_PIN) == HIGH) << 1;
		key_state |= uint8_t(digitalRead(K3_PIN) == HIGH) << 2;
		key_state |= uint8_t(digitalRead(K4_PIN) == HIGH) << 3;
		if(key_state) {
			Serial.println("Reset gotosleep timer");
			reset_gotosleep_timer();
		}
				Serial.print("keys: ");
				Serial.println(key_state, HEX);
				Serial.print("key 1: ");
				Serial.println(digitalRead(K1_PIN));
	});
	reset_gotosleep_timer();
}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(is_wifi_connected())
	{
		httpsrv::update();
		if(udp.connected()) {
			time_t t = time(NULL);
			if(refresh_screen_ts + REFRESH_SCREEN_TIMEOUT < t) {
				refresh_screen_ts = t;
				tft.setTextSize(1);
				tft.setFont(&FreeMonoBoldOblique12pt7b);
				switch(screen_num) {
					case VAL_SCREEN:
						if(read_ts != display_ts) {
							display_ts = read_ts;
							int y = 30;
							int y_step = 30;
							tft.fillScreen(ST77XX_BLACK);
							tft.setCursor(0, y);
							tft.setTextColor(ST77XX_RED);
							tft.print("t:");
							tft.println(t);
							y += y_step;
							tft.setCursor(0, y);
							tft.setTextColor(ST77XX_BLUE);
							tft.print("h:");
							tft.println(h);
							y += y_step;
							tft.setCursor(0, y);
							tft.setTextColor(ST77XX_GREEN);
							tft.print("co2:");
							tft.println(co2);
							y += y_step;
							tft.setCursor(0, y);
							tft.setTextColor(ST77XX_WHITE);
							tft.print("keys: ");
							tft.println(key_state, HEX);
							tft.print(", sd: ");
							tft.println(SD_available);
						}
						break;
					default:
						break;
				}
			}
		} else {
			connectUDP();
		}
		if(do_check_for_better_wifi) {
			do_check_for_better_wifi = false;
			ap_t better_ap = find_better_ap(-20);
			if(better_ap.ssid.length() > 0) {
				Serial.print("Better found:");
				Serial.print(better_ap.ssid);
				Serial.println(", disconnecting..");
				WiFi.disconnect(false, false);
			}
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
				get_data_now();
				httpsrv::init();
			} else {
				Serial.println("Error setting up MDNS responder!");
			}
		}
	}
	delay(10);
}
