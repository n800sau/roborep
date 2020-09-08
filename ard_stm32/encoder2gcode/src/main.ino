#include <USBComposite.h>
#include <RotaryEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define IN_USBSER 0
#define OUT_USBSER 1

USBMultiSerial<2> ms;

#define CNCSerial Serial1
#define DBGSerial Serial

int16_t x_position = 0, y_position = 0, z_position=0;

// p1, p2, button
const int xp1=PB13, xp2=PB14, xb=PB12;
RotaryEncoder encoderX(xp1, xp2, xb);

const int yp1=PB3, yp2=PB4, yb=PA15;
RotaryEncoder encoderY(yp1, yp2, yb);

const int zp1=PB6, zp2=PB7, zb=PB5;
RotaryEncoder encoderZ(zp1, zp2, zb);

#define TFT_CS PB11
#define TFT_DC PB10
#define TFT_RST PB1
#define TFT_MOSI PB0
#define TFT_SCLK PA7

// colored 160x80 0.96"
Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
GFXcanvas1 dbuf(160, 80);

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
	dbuf.setCursor(0, 0);
	dbuf.print(F("X:"));
	dbuf.print(x_position);
	dbuf.setCursor(0, 20);
	dbuf.print(F("Y"));
	dbuf.print(y_position);
	dbuf.setCursor(0, 40);
	dbuf.print(F("Z"));
	dbuf.print(z_position);
	display.drawBitmap(0, 0, dbuf.getBuffer(), dbuf.width(), dbuf.height(), ST77XX_YELLOW, ST77XX_BLACK);
}

void setup()
{

	DBGSerial.begin(115200);
	display.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
	display.setTextWrap(false); // Allow text to run off right edge
	display.fillScreen(ST7735_BLACK);
	display.setRotation(1);
	display.setTextSize(1);
	display.setTextColor(ST77XX_WHITE);
	update_display();

	ms.begin();
	while(!USBComposite);


	encoderX.begin();
	encoderY.begin();
	encoderZ.begin();

	attachInterrupt(digitalPinToInterrupt(xp1), XencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(xb), XencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(yp1), YencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(yb), YencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(zp1), ZencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(zb), ZencoderButtonISR, FALLING);

	CNCSerial.begin(115200);
}

void loop()
{

	bool changed = false;

	if (x_position != encoderX.getPosition())
	{
		changed = true;
		x_position = encoderX.getPosition();
		DBGSerial.print("Xpos:");
		DBGSerial.println(x_position);
	}
	
	if (encoderX.getPushButton() == true)
	{
		DBGSerial.println(F("X PRESSED"));
	}

	if (y_position != encoderY.getPosition())
	{
		changed = true;
		y_position = encoderY.getPosition();
		DBGSerial.print("Ypos:");
		DBGSerial.println(y_position);
	}
	
	if (encoderY.getPushButton() == true)
	{
		DBGSerial.println(F("Y PRESSED"));
	}

	if (z_position != encoderZ.getPosition())
	{
		changed = true;
		z_position = encoderZ.getPosition();
		DBGSerial.print("Zpos:");
		DBGSerial.println(z_position);
	}
	
	if (encoderZ.getPushButton() == true)
	{
		DBGSerial.println(F("Z PRESSED"));
	}

	if(changed) {
		update_display();
	}

	while(ms.ports[IN_USBSER].available())
	{
		ms.ports[OUT_USBSER].write(ms.ports[0].read());
	}

}
