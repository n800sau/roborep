//#include "thermistor.h"
#include <RTClock.h>

#define REFERENCE_RESISTANCE   330000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950
#define STM32_ANALOG_RESOLUTION 4095

#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

// ST7789 TFT module connections
#define TFT_CS     PA4
#define TFT_RST    PA2
#define TFT_DC     PA3
#define TFT_SCLK   PA5
#define TFT_MOSI   PA7

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define PLOT_TOP 24
#define PLOT_BOTTOM 239
#define PLOT_LEFT 24
#define PLOT_HEIGHT (PLOT_BOTTOM - PLOT_TOP)
#define PIXELS_PER_STEP 12

GFXcanvas1 dbuf(240, PLOT_TOP);

// unknown temp marker
const int UNKNOWN_TEMP = -1000;

double temp = UNKNOWN_TEMP;

// NTC
const int TEMP_PIN = PB1;
// 2.5v ref
const int REF_PIN = PB0;

#define TEMP_HISTORY_SIZE (240-PLOT_LEFT)
int8_t temp_history[TEMP_HISTORY_SIZE];
uint16_t temp_history_count = 0;
int32_t plot_max_temp = 100;
int32_t plot_min_temp = 10;

float Vcc = 3.3;
// ref from LM336-2.5
float Vref = 2.45;
const float T_0 = 273.15;
const float T_25 = T_0 + 25;
// 100k NTC 110C ~ 5kom, 0C ~ 327.24kom
const float beta = 3950;
const float R_25 = 100000L; // 100k ohm
//const unsigned long Rs = 10000L;
const unsigned long Rs = 100000L;
//const unsigned long Rs = 330000L;
//const unsigned long Rs = 470000L;
// ************************************************

RTClock rtclock (RTCSEL_LSE); // initialise

void draw_plot_scale()
{
	int ptemp = plot_min_temp;
	//PLOT_HEIGHT
	int pstep = 20;
	while(ptemp < plot_max_temp) {
		ptemp += pstep;
		if(ptemp != plot_min_temp) {
			int y = map(ptemp, plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP);
//			Serial.print(ptemp);
//			Serial.print(F(" at "));
//			Serial.println(y);
			display.setCursor(0, y);
			display.print(ptemp);
		}
	}
}

// show current process
void display_status()
{
	dbuf.fillScreen(0);
	if(temp != UNKNOWN_TEMP) {
		dbuf.setCursor(30, 0);
		dbuf.print(F("="));
		dbuf.print(temp);
		dbuf.print(F("C"));
		Serial.print(F("T "));
		Serial.println(temp);
	}
	// copy buffer to screen
	display.drawBitmap(0, 0, dbuf.getBuffer(), dbuf.width(), dbuf.height(), ST77XX_YELLOW, ST77XX_BLACK);
//	Serial.print(F("history size:"));
//	Serial.println(temp_history_count);
//	Serial.print(F("min temp:"));
//	Serial.println(plot_min_temp);
//	Serial.print(F("max temp:"));
//	Serial.println(plot_max_temp);
	display.drawLine(PLOT_LEFT, PLOT_TOP, PLOT_LEFT, PLOT_BOTTOM, ST77XX_BLACK);
	for(int i=1; i<temp_history_count; i++) {
		// fill line black
		display.drawLine(PLOT_LEFT+i, PLOT_TOP, PLOT_LEFT+i, PLOT_BOTTOM, ST77XX_BLACK);
		display.drawLine(PLOT_LEFT+i-1, map(temp_history[i-1], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), PLOT_LEFT+i,
				map(temp_history[i], plot_min_temp-1, plot_max_temp+1, PLOT_BOTTOM, PLOT_TOP), ST77XX_RED);
	}
}

void update_display()
{
	display_status();
}

// NTC scheme
// GND - NTC - measure - Rs - Vref
double ntcVoltageRead(int temp_pin)
{
	double val = 0;
	double ref = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(temp_pin);
		ref += analogRead(REF_PIN);
		delay(1);
	}
//	Serial.print(F("val:"));
//	Serial.print(val);
//	Serial.print(F(", ref:"));
//	Serial.print(ref);
//	Serial.print(F(", ref V:"));
//	Serial.print(ref/4096/20*Vcc);
//	Serial.print(F(", NCC V:"));
//	Serial.println(val/4096/20*Vcc);
	// Voltage on NTC
	return Vref * val / ref;
}

double read_temp(int temp_pin)
{
	double v = ntcVoltageRead(temp_pin);
	// voltage on Rs
	double rV = Vref-v;
	// current
	double current = rV / Rs;
	// resistance of NTC
	double r = v / current;
	// temp on NTC
	double rs = 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
//	Serial.print(F("R:"));
//	Serial.println(r);
//	Serial.print(F("V:"));
//	Serial.println(v);
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return rs;
}

void update_temp()
{
	if(temp == UNKNOWN_TEMP) {
		Serial.println("Ready");
	}
	temp = read_temp(TEMP_PIN);
	//temp = thermistor.readCelsius();

//	Serial.print(F("Temp:"));
//	Serial.println(temp);
}

void update_history_proc()
{
	if(temp != UNKNOWN_TEMP) {
		if(temp_history_count >= TEMP_HISTORY_SIZE) {
			for(int i=1; i<TEMP_HISTORY_SIZE; i++) {
				temp_history[i-1] = temp_history[i];
			}
			temp_history_count--;
		}
		temp_history[temp_history_count] = temp;
		temp_history_count++;
	}
}

Ticker status_timer(update_display, 1000, 0, MILLIS);
Ticker temp_history_timer(update_history_proc, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	pinMode(TEMP_PIN, INPUT);
	pinMode(REF_PIN, INPUT);

	display.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
	display.setRotation(2);
	display.fillScreen(ST77XX_BLACK);
	display.setTextSize(1);
	display.setTextColor(ST77XX_WHITE);
	draw_plot_scale();

	temp_history_timer.start();
	status_timer.start();
}

void loop()
{
	temp_history_timer.update();
	status_timer.update();
}
