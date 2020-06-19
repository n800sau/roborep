#include "WiFi.h"
#include <ESPmDNS.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <SD.h>
#include <blinker.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <SPI.h>

#include "config.h"
const char * ssid = WIFI_SSID;
const char * password = WIFI_PASSWORD;

#define K1_PIN 15
#define K2_PIN 2
#define K3_PIN 4
#define K4_PIN 16

// set it high to go to sleep
#define GOTOSLEEP_PIN 18

volatile uint8_t key_state = 0;

// SPI for SD pins
// miso - 12
// mosi - 13
// clk - 14
// chosen ss pin - 25

#define SD_SS 25
// default SPI pins
//#define SPI_MISO 12
//#define SPI_MOSI 13
//#define SPI_CLK  14

bool SD_available = false;

#define TFT_CS   35  // old - 13
#define TFT_RST  26  // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC   32  //A0
#define TFT_MOSI 33  // SDA Data out
#define TFT_SCLK 27  // SCK Clock out

#define LED_PIN 11
Blinker blinker;
Ticker check_wifi_ticker;
Ticker keyboard_ticker;
Ticker gotosleep_ticker;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
GFXcanvas1 dbuf(180, 120);

#define HOSTNAME "esp32display"

AsyncUDP udp;

//StaticJsonDocument<400> doc;
DynamicJsonDocument doc(400);

volatile time_t read_ts = 0, display_ts = 0;
float t, h, co2;

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
	gotosleep_ticker.once(30, []() {
		digitalWrite(GOTOSLEEP_PIN, HIGH);
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

void setup()
{
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

	pinMode(GOTOSLEEP_PIN, OUTPUT);
	digitalWrite(GOTOSLEEP_PIN, LOW);

	SD_available = SD.begin(SD_SS);
	if(!SD_available) {
		Serial.println("Card Mount Failed");
	}

	tft.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
	tft.fillScreen(ST77XX_BLACK);
	tft.setFont(&FreeMonoBoldOblique12pt7b);
	tft.setTextSize(0.5);
	tft.setCursor(0, 30);
	tft.setTextColor(ST77XX_WHITE);
	Serial.print(F("TFT initialized at "));
	Serial.println(millis()/1000);

	WiFi.mode(WIFI_STA);
	check_wifi_ticker.attach(30, [](){ do_check_for_better_wifi=true; });
	keyboard_ticker.attach(1, []() {
		key_state = 0;
		key_state |= uint8_t(digitalRead(K1_PIN) == LOW);
		key_state |= uint8_t(digitalRead(K2_PIN) == LOW) << 1;
		key_state |= uint8_t(digitalRead(K3_PIN) == LOW) << 2;
		key_state |= uint8_t(digitalRead(K4_PIN) == LOW) << 3;
		reset_gotosleep_timer();
	});
	reset_gotosleep_timer();
}

void loop()
{
	// check if the WiFi and UDP connections were successful
	if(is_wifi_connected())
	{
//		server.handleClient();
		if(udp.connected()) {
			if(read_ts != display_ts) {
				int y = 30;
				int y_step = 30;
				display_ts = read_ts;
				tft.fillScreen(ST77XX_BLACK);
				tft.setTextSize(1);
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
				tft.print("keyb:");
				tft.println(key_state, HEX);
				Serial.print("keys: ");
				Serial.print(key_state, HEX);
				Serial.print(", sd: ");
				Serial.println(SD_available);
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
			} else {
				Serial.println("Error setting up MDNS responder!");
			}
		}
	}
	delay(10);
}
