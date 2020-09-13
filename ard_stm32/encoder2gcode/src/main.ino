#include <USBComposite.h>
#include <RotaryEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define USBSerial 0

// USB - PA11,PA12
USBMultiSerial<1> ms;

#define CNCSerial Serial1
#define DBGSerial Serial

int16_t x_pos = 0, y_pos = 0, z_pos = 0;
float x_fpos = 0, y_fpos = 0, z_fpos = 0;
float x_frate = 1, y_frate = 1, z_frate = 1;

enum eMODE {EM_POS, EM_RATE};

// p1, p2, button
const int xp1=PB13, xp2=PB14, xb=PB12;
RotaryEncoder encoderX(xp1, xp2, xb);
eMODE xMode = EM_POS;

const int yp1=PA3, yp2=PA2, yb=PA4;
RotaryEncoder encoderY(yp1, yp2, yb);
eMODE yMode = EM_POS;

const int zp1=PB8, zp2=PB9, zb=PB7;
RotaryEncoder encoderZ(zp1, zp2, zb);
eMODE zMode = EM_POS;

#define TFT_CS PB15
#define TFT_DC PB1
#define TFT_RST PB0
#define TFT_MOSI PA7
#define TFT_SCLK PA5

// colored 160x80 0.96"
//Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
// SPI version much faster
SPIClass SPI_1(1);
Adafruit_ST7735 display = Adafruit_ST7735(&SPI_1, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas1 dbuf(160, 80);

#define LED_PIN LED_BUILTIN

void XencoderISR()
{
	encoderX.readAB();
}

void XencoderButtonISR()
{
	encoderX.readPushButton();
}

void YencoderISR()
{
	encoderY.readAB();
}

void YencoderButtonISR()
{
	encoderY.readPushButton();
}

void ZencoderISR()
{
	encoderZ.readAB();
}

void ZencoderButtonISR()
{
	encoderZ.readPushButton();
}

void update_display()
{
	dbuf.fillScreen(0);
	int y = 5, x[] = {10, 80};
	const int y_step = 20;
	dbuf.setCursor(x[0], y);
	dbuf.print(F("X: "));
	dbuf.print(x_fpos);
	dbuf.print(F(","));
	dbuf.setCursor(x[1], y);
	dbuf.print(xMode ? F("*") : F(" "));
	dbuf.print(x_frate);
	y += y_step;
	dbuf.setCursor(x[0], y);
	dbuf.print(F("Y: "));
	dbuf.print(y_fpos);
	dbuf.print(F(","));
	dbuf.setCursor(x[1], y);
	dbuf.print(yMode ? F("*") : F(" "));
	dbuf.print(y_frate);
	y += y_step;
	dbuf.setCursor(x[0], y);
	dbuf.print(F("Z :"));
	dbuf.print(z_fpos);
	dbuf.print(F(","));
	dbuf.setCursor(x[1], y);
	dbuf.print(zMode ? F("*") : F(" "));
	dbuf.print(z_frate);
	display.drawBitmap(0, 0, dbuf.getBuffer(), dbuf.width(), dbuf.height(), ST7735_YELLOW, ST7735_BLACK);
}

void setup()
{
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	SPI_1.begin(); //Initialize the SPI_2 port.
//	SPI_1.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
//	SPI_1.setDataMode(SPI_MODE0); //Set the	SPI_2 data mode 0
//	SPI_1.setClockDivider(SPI_CLOCK_DIV16);	// Use a different speed to SPI 1

	DBGSerial.begin(115200);
	DBGSerial.println("Hello from encoder2gcode");
	DBGSerial.flush();

	display.initR(INITR_MINI160x80); // initialize a ST7735S chip
	display.setTextWrap(false); // Allow text to run off right edge
	display.fillScreen(ST7735_BLACK);
	display.setRotation(1);
	display.setTextSize(1);
	display.invertDisplay(true);
	display.setTextColor(ST77XX_WHITE);
	update_display();

	ms.begin();
	//while(!USBComposite);


	encoderX.begin();
	encoderY.begin();
	encoderZ.begin();

	attachInterrupt(digitalPinToInterrupt(xp1), XencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(xp2), XencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(xb), XencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(yp1), YencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(yp2), YencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(yb), YencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(zp1), ZencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(zp2), ZencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(zb), ZencoderButtonISR, FALLING);

	CNCSerial.begin(115200);
}

bool encoder_update(RotaryEncoder &enc, int16_t &oldpos, eMODE mode, float &fpos, float &frate)
{
	int16_t pos = enc.getPosition();

	float pdiff = pos - oldpos;
	if(mode == EM_RATE) {
		frate += frate * pdiff/10.;
		if(frate < 0.1) {
			frate = 0.1;
		}
	} else {
		fpos += pdiff * frate;
	}
	oldpos = pos;
	return bool(pdiff);
}

unsigned long m = millis();

void loop()
{

	bool changed = false;

	if(encoderX.getPushButton()) {
		changed = true;
		Serial.println("X pressed");
		xMode = xMode == EM_POS ? EM_RATE : EM_POS;
	}

	if(encoder_update(encoderX, x_pos, xMode, x_fpos, x_frate))
	{
		changed = true;
		DBGSerial.print("Xpos:");
		DBGSerial.println(x_fpos);
		DBGSerial.print("Xrate:");
		DBGSerial.println(x_frate);
	}

	if(encoderY.getPushButton()) {
		changed = true;
		yMode = yMode == EM_POS ? EM_RATE : EM_POS;
		Serial.println("Y pressed");
	}

	if(encoder_update(encoderY, y_pos, yMode, y_fpos, y_frate))
	{
		changed = true;
		DBGSerial.print("Ypos:");
		DBGSerial.println(y_fpos);
		DBGSerial.print("Yrate:");
		DBGSerial.println(y_frate);
	}

	if(encoderZ.getPushButton() == true) {
		changed = true;
		zMode = zMode == EM_POS ? EM_RATE : EM_POS;
		Serial.println("Z pressed");
	}

	if(encoder_update(encoderZ, z_pos, zMode, z_fpos, z_frate))
	{
		changed = true;
		DBGSerial.print("Zpos:");
		DBGSerial.println(z_fpos);
		DBGSerial.print("Zrate:");
		DBGSerial.println(z_frate);
	}

	DBGSerial.flush();
	if(changed) {
		Serial.println("Update display");
		update_display();
		Serial.println("Update end");
	}

	while(ms.ports[USBSerial].available() || CNCSerial.available()) {
		if(ms.ports[USBSerial].available())
		{
			CNCSerial.write(ms.ports[USBSerial].read());
		}
		if(CNCSerial.available()) {
			ms.ports[USBSerial].write(CNCSerial.read());
		}
	}

	if(m+1000 < millis()) {
		m = millis();
		digitalWrite(LED_PIN, !digitalRead(LED_PIN));
	}
}
