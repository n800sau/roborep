#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>
#include <EEPROM.h>
#include "grideye.h"

#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0xffff ^ 0x0000
#define	BLUE    0xffff ^ 0x001F
#define	RED     0xffff ^ 0xF800
#define	GREEN   0xffff ^ 0x07E0
#define CYAN    0xffff ^ 0x07FF
#define MAGENTA 0xffff ^ 0xF81F
#define YELLOW  0xffff ^ 0xFFE0
#define WHITE   0xffff ^ 0xFFFF

const int TS_MINPRESSURE = 10;
const int TS_MAXPRESSURE = 1000;

const int TS_MINX=201;
const int TS_MINY=190;
const int TS_MAXX=996;
const int TS_MAXY=937;


Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

const int BOXSIZE = 30;

int const MIN_HUE = 230;
int const MAX_HUE = 360;

int min_temp = 10;
int max_temp = 40;

const int MARKER = 0xA3F1;
const int marker_addr = 0;
const int min_temp_addr = marker_addr + sizeof(MARKER);
const int max_temp_addr = min_temp_addr + sizeof(min_temp);

bool led_state = false;

const int RIGHT_PANE_OFF = BOXSIZE * 8;

void setup(void) {
	// serial rate of GridEye Kit
	Serial.begin(115200);

	grideye::setup();

	Serial.println(F("GridEye show"));

	memset(terms, 0, sizeof(terms));

	int t;
	EEPROM.get(marker_addr, t);
	if(t == MARKER) {
		EEPROM.get(min_temp_addr, min_temp);
		EEPROM.get(max_temp_addr, max_temp);
	}

	tft.reset();

	tft.begin(0x9341); // SDFP5408

	tft.setRotation(3); // Need for the Mega, please changed for your choice or rotation initial

	// Border

//	drawBorder();
	
//	waitOneTouch();

	tft.fillScreen(BLACK);

 
	pinMode(13, OUTPUT);

	tft.setTextSize(1);

	tft.setCursor(RIGHT_PANE_OFF + 35, 20);
	tft.setTextColor(RED);
	tft.print("MAX");

	tft.setTextColor(WHITE);
	tft.setCursor(RIGHT_PANE_OFF + 40, 50);
	tft.print("+");
	tft.setCursor(RIGHT_PANE_OFF + 40, 100);
	tft.print("-");

	tft.setCursor(RIGHT_PANE_OFF + 35, 130);
	tft.setTextColor(BLUE);
	tft.print("MIN");

	tft.setTextColor(WHITE);
	tft.setCursor(RIGHT_PANE_OFF + 40, 160);
	tft.print("+");
	tft.setCursor(RIGHT_PANE_OFF + 40, 215);
	tft.print("-");

	draw_min_max();
}

int temp = 5;

int temp2hue(int temp)
{
	return MIN_HUE + (MAX_HUE - MIN_HUE) * long(temp - min_temp) / (max_temp - min_temp);
}

void store2eeprom()
{
	EEPROM.put(marker_addr, MARKER);
	EEPROM.put(min_temp_addr, min_temp);
	EEPROM.put(max_temp_addr, max_temp);
}

void draw_min_max()
{
	tft.setTextColor(WHITE);

	tft.fillRect(RIGHT_PANE_OFF + 35, 75, 20, 20, BLACK);
	tft.setCursor(RIGHT_PANE_OFF + 35, 75);
	tft.print(String(max_temp));

	tft.fillRect(RIGHT_PANE_OFF + 35, 190, 20, 20, BLACK);
	tft.setCursor(RIGHT_PANE_OFF + 35, 190);
	tft.print(String(min_temp));

}

void test_touch()
{
	TSPoint p = ts.getPoint(); 
	pinMode(XM, OUTPUT); //Pins configures again for TFT control
	pinMode(YP, OUTPUT);
	if(p.z > TS_MINPRESSURE && p.z < TS_MAXPRESSURE) {
		if(p.y < 400 && p.y > 230) {
			if(p.x > 150 && p.x < 350) {
				max_temp += 1;
			} else if(p.x > 400 && p.x < 500) {
				max_temp -= 1;
			} else if(p.x > 600 && p.x < 700) {
				min_temp += 1;
			} else if(p.x > 800 && p.x < 900) {
				min_temp -= 1;
			}
			store2eeprom();
			draw_min_max();
		}
	}
}

void loop()
{

	byte r, g, b;
	int color, hue;

	if(grideye::refresh_terms()) {
		for(int col=0; col<8; col++) {
			for(int row=0; row<8; row++) {
				hue = temp2hue(terms[row][col]);
				HSV2RGB(hue, 255, 255, r, g, b);
				color = 0xffff ^ tft.color565(r, g, b);
				tft.fillRect(col*BOXSIZE+1, row*BOXSIZE+1, BOXSIZE-2, BOXSIZE-2, color);
				tft.setCursor(col*BOXSIZE+10, row*BOXSIZE+10);
				tft.setTextSize(1);
				tft.setTextColor(BLACK);
				tft.print(String(int(terms[row][col])));
//				tft.print(String(int(terms[row][col])) + "." + String(int(terms[row][col]*10) % 10));
				Serial.print(int(terms[row][col]), DEC);
				Serial.print(" ");
				test_touch();
			}
			Serial.println();
		}
		Serial.println();
	} else {
		test_touch();
		Serial.println("skip");
	}

//	Serial.print("Temp:");
//	Serial.print(temp);
//	Serial.print("Hue:");
//	Serial.print(hue);
//	Serial.print(", R:");
//	Serial.print(r);
//	Serial.print(", G:");
//	Serial.print(g);
//	Serial.print(", B:");
//	Serial.println(b);
	temp += 2;
	if(temp >= max_temp + 5) {
		temp = min_temp;
	}
	digitalWrite(13, led_state ? HIGH : LOW);
	led_state = !led_state;
	delay(100);
}

// Wait one touch

void drawBorder () {

	// Draw a border

	uint16_t width = tft.width() - 1;
	uint16_t height = tft.height() - 1;
	uint8_t border = 10;

	tft.fillScreen(RED);
	tft.fillRect(border, border, (width - border * 2), (height - border * 2), WHITE);
	
}


