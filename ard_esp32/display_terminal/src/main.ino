#include "WiFi.h"
#include <ESPmDNS.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <SD.h>
#include <blinker.h>
#include <HTTPClient.h>
#include <StreamString.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeMonoBoldOblique9pt7b.h>
#include <Fonts/FreeMonoBoldOblique30pt7b.h>
#include <Fonts/Org_01.h>
#include <SPI.h>

#include "httpsrv.hpp"

#include "config.h"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define K1_PIN 26 // "1"
#define K2_PIN 27 // "2"
#define K3_PIN 25 // "3"
#define K4_PIN 33 // "4"

// set it high to go to sleep
#define GOTOSLEEP_REQUEST_PIN 14
#define GOTOSLEEP_TIMEOUT 60

volatile uint8_t key_state = 0;

#define TFT_CS         4
#define TFT_RST        22
#define TFT_DC         21

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
enum D_VIEW {DV_TEMP, DV_HUM, DV_CO2, DV_INFO, DV_COUNT};
D_VIEW d_view = DV_TEMP;

#define LED_PIN 34
Blinker blinker;
Ticker check_wifi_ticker;
Ticker keyboard_ticker;
Ticker gotosleep_ticker;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#define DISPLAY_WIDTH 160
#define DISPLAY_HEIGHT 128

#define HOSTNAME "esp32display"

AsyncUDP udp;
HTTPClient http;
String sensor_data;

volatile time_t read_ts = 0, display_ts = 0, refresh_screen_ts = 0;
float t, h, co2;
#define REFRESH_SCREEN_TIMEOUT 1

#define TS_FORMAT "%Y-%m-%d %H:%M:%S"

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

ap_t find_better_ap(int thresh=20, bool verbose=false)
{
	ap_t rs;
	int max_power = 0;
	String current_ssid;
	if(is_wifi_connected()) {
		max_power = WiFi.RSSI();
		current_ssid = WiFi.SSID();
		Serial.print("Connected to ");
		Serial.println(current_ssid);
	} else {
		thresh = 0;
	}
	int n = WiFi.scanNetworks();
	if(n > 0) {
		Serial.print(n);
		Serial.println(" networks found");
		for (int i = 0; i < n; ++i) {
			if(verbose) {
				// Print SSID and RSSI for each network found
				Serial.print(i + 1);
				Serial.print(": ");
				Serial.print(WiFi.SSID(i));
				Serial.print(" (");
				Serial.print(WiFi.RSSI(i));
				Serial.println(")");
			}
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

void print_uptime_prefix(Print &printer=Serial)
{
	printer.print("[");
	printer.print(millis());
	printer.print("]");
}

void print_ts(Print &printer=Serial)
{
	struct tm timeinfo;
	if(getLocalTime(&timeinfo)) {
		printer.print(&timeinfo, TS_FORMAT);
	}
}

void print_ts_prefix(Print &printer=Serial)
{
	printer.print("[");
	print_ts(printer);
	printer.print("]");
}

void connectWifi()
{
	static bool connection_success = true;
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
		connection_success = true;
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(WiFi.SSID());
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
		if(connection_success) {
			Serial.println("Connection failed.");
		}
		connection_success = false;
		delay(100);
	}
}

void serial_print_data()
{
	Serial.print("t:");
	Serial.print(t);
	Serial.print(", h:");
	Serial.print(h);
	Serial.print(", co2:");
	Serial.print(co2);
	Serial.printf(",Free mem: %d", ESP.getFreeHeap());
	Serial.printf(",Free sketch space: %d", ESP.getFreeSketchSpace());
}

void process_json()
{
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
				co2 = doc["co2"];
				read_ts = time(NULL);
				print_ts_prefix();
				Serial.print(": Sensor:");
				Serial.println((const char *)doc["sensor_id"]);
			}
		}
		sensor_data = "";
	}
}

void connectUDP()
{
	Serial.print("UDP listen");
	if(udp.listenMulticast(IPAddress(239, 0, 0, 57), 8989)) {
		Serial.print("UDP Listening on IP: ");
		Serial.println(WiFi.localIP());
		udp.onPacket([](AsyncUDPPacket packet) {
			String data = packet.readString();
/*
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
			Serial.print(", Data: ");
			Serial.print(data);
//			Serial.write(packet.data(), packet.length());
			Serial.println();
*/
			//reply to the client
			packet.printf("Got %u bytes of data", packet.length());
			sensor_data = data.c_str();
		});
	}
}

void reset_gotosleep_timer()
{
//	Serial.println("Reset gotosleep timer");
	digitalWrite(GOTOSLEEP_REQUEST_PIN, LOW);
	gotosleep_ticker.once(GOTOSLEEP_TIMEOUT, []() {
		print_uptime_prefix();
		Serial.println(" go to sleep");
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
	reset_gotosleep_timer();
	http.begin("http://mq135.local/data.json");
	http.addHeader("Content-Type","text/json");
	int httpResponceCode = http.GET();
	if (httpResponceCode == 200) {
		Serial.print("get_data_now:");
		Serial.println(httpResponceCode);
		String response = http.getString();
//		Serial.println(response);
		if(response.length() > 0) {
			sensor_data = response;
		}
		update_display();
	} else {
		Serial.print("Direct connection to sensor failed: ");
		Serial.print(httpResponceCode);
		Serial.println("(" + http.errorToString(httpResponceCode) + ")");
	}
//	http.end();
}

void print_text_bounds(String text, int16_t x, int16_t y)
{
	int16_t xp, yp;
	uint16_t w, h;
	tft.getTextBounds(text, x, y, &xp, &yp, &w, &h);
	Serial.printf("bounds of %s:", text.c_str());
	Serial.print(xp);
	Serial.print(",");
	Serial.print(yp);
	Serial.print(",");
	Serial.print(w);
	Serial.print("x");
	Serial.println(h);
}

void setup()
{
	Serial.begin(115200);
	Serial.println();
	print_uptime_prefix();
	Serial.println("setup");

	blinker.begin(LED_PIN);
	digitalWrite(LED_PIN, HIGH);

	pinMode(K1_PIN, INPUT_PULLUP);
	pinMode(K2_PIN, INPUT_PULLUP);
	pinMode(K3_PIN, INPUT_PULLUP);
	pinMode(K4_PIN, INPUT_PULLUP);

	pinMode(GOTOSLEEP_REQUEST_PIN, OUTPUT);
	digitalWrite(GOTOSLEEP_REQUEST_PIN, LOW);

//	SD_available = SD.begin(SD_SS);
//	if(!SD_available) {
//		Serial.println("Card Mount Failed");
//	}

	tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
	tft.fillScreen(ST77XX_BLACK);
	tft.setRotation(3);
	tft.setFont(&FreeMonoBoldOblique9pt7b);
	tft.setTextSize(1);
	tft.setTextColor(ST77XX_WHITE);
	tft.setCursor(0, 30);
	center_align_print("Waiting...");

	WiFi.mode(WIFI_STA);
	check_wifi_ticker.attach(120, [](){ do_check_for_better_wifi=true; });
	keyboard_ticker.attach(1, []() {
		uint8_t new_key_state = 0;
		new_key_state |= uint8_t(digitalRead(K1_PIN) == HIGH);
		new_key_state |= uint8_t(digitalRead(K2_PIN) == HIGH) << 1;
		new_key_state |= uint8_t(digitalRead(K3_PIN) == HIGH) << 2;
		new_key_state |= uint8_t(digitalRead(K4_PIN) == HIGH) << 3;
		if(new_key_state && new_key_state != key_state) {
			key_state = new_key_state;
			reset_gotosleep_timer();
			for(int i=0; i<4; i++) {
				int b = 1 << i;
				if(key_state & b) {
					d_view = (D_VIEW)i;
				}
			}
			update_display();
		}
//				Serial.print("keys: ");
//				Serial.println(key_state, BIN);
//				Serial.print("key 1: ");
//				Serial.println(digitalRead(K1_PIN));
	});
	if(MDNS.begin(HOSTNAME)) {
		Serial.println("MDNS responder " HOSTNAME " started");
		// Add service to MDNS-SD
		MDNS.addService("http", "tcp", 80);
//		httpsrv::init();
	} else {
		Serial.println("Error setting up MDNS responder!");
	}
	reset_gotosleep_timer();
}

void center_align_print(String text)
{
	int16_t xp, yp;
	uint16_t w, h;
	tft.getTextBounds(text, 0, 0, &xp, &yp, &w, &h);
//	Serial.print("init pos:");
//	Serial.print(xp);
//	Serial.print(",");
//	Serial.println(yp);
	xp = (DISPLAY_WIDTH - w) / 2;
	yp = (DISPLAY_HEIGHT - h)/2 + h;
//	Serial.print("pos:");
//	Serial.print(xp);
//	Serial.print(",");
//	Serial.println(yp);
//	print_text_bounds(text, xp, yp);
	tft.setCursor(xp, yp);
	tft.print(text);
}

void update_display()
{
	StreamString text;
	print_uptime_prefix();
	Serial.println("update display");
	process_json();
	serial_print_data();
	Serial.println();
	switch(d_view) {
		case DV_TEMP:
			{
				tft.setFont(&FreeMonoBoldOblique30pt7b);
				tft.fillScreen(ST77XX_BLACK);
				tft.setTextColor(ST77XX_RED);
				text.printf("%d", (int)t);
				center_align_print(text);
//				tft.drawRect(0, 0, DISPLAY_WIDTH-1, DISPLAY_HEIGHT-1, ST77XX_BLUE);
			}
			break;
		case DV_HUM:
			{
				tft.setFont(&FreeMonoBoldOblique30pt7b);
				tft.fillScreen(ST77XX_BLACK);
				tft.setTextColor(ST77XX_BLUE);
				text.printf("%d", (int)h);
				center_align_print(text);
			}
			break;
		case DV_CO2:
			{
				tft.setFont(&FreeMonoBoldOblique30pt7b);
				tft.fillScreen(ST77XX_BLACK);
				tft.setTextColor(ST77XX_GREEN);
				text.printf("%.1f", co2);
				center_align_print(text);
			}
			break;
		default:
			{
				int y_step = 30;
				int y = 20;
				tft.fillScreen(ST77XX_BLACK);
				tft.setFont(&Org_01);
				tft.setCursor(0, y);
				tft.setTextColor(ST77XX_CYAN);
				print_ts(tft);
				tft.setFont(&FreeMonoBoldOblique9pt7b);
				tft.setCursor(0, y);
				tft.setTextColor(ST77XX_RED);
				tft.printf("%d", (int)t);
				tft.setTextColor(ST77XX_BLUE);
				tft.printf(" %d", (int)h);
				tft.setTextColor(ST77XX_GREEN);
				tft.printf(" %.1f", co2);
				y += y_step;
				tft.setCursor(0, y);
				tft.setTextColor(ST77XX_YELLOW);
				tft.setFont(&Org_01);
				tft.print(WiFi.SSID());
				y += y_step;
				tft.setCursor(0, y);
				tft.setTextColor(ST77XX_MAGENTA);
				tft.printf("sd: %s%s", SD_available ? "" : " not"," available");
				y += y_step;
				tft.setCursor(0, y);
				tft.setTextColor(ST77XX_GREEN);
				tft.printf("Free mem: %d", ESP.getFreeHeap());
			}
			break;
	}
}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(is_wifi_connected())
	{
		httpsrv::update();
		if(udp.connected()) {
			time_t tm = time(NULL);
			if(refresh_screen_ts + REFRESH_SCREEN_TIMEOUT < tm) {
				refresh_screen_ts = tm;
				if(read_ts != display_ts) {
					display_ts = read_ts;
					update_display();
				}
			}
		} else {
			connectUDP();
		}
		if(do_check_for_better_wifi) {
			do_check_for_better_wifi = false;
//			WiFi.disconnect(false, false);
//			ap_t better_ap = find_better_ap(-20);
//			if(better_ap.ssid.length() > 0) {
//				Serial.print("Better found:");
//				Serial.print(better_ap.ssid);
//				Serial.println(", disconnecting..");
//				WiFi.disconnect(false, false);
//			}
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
			if(h == 0) {
				// request data the first time only
				get_data_now();
			}
		}
	}
	delay(10);
}
