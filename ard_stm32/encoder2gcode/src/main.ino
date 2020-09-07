#include <USBComposite.h>
#include <RotaryEncoder.h>

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

void setup()
{
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

	DBGSerial.begin(115200);
	CNCSerial.begin(115200);
}

void loop()
{

	int inByte;

	while(Serial1.available() || Serial2.available()) {
		if(Serial1.available()) {
			inByte = Serial1.read();
			Serial2.write(inByte);
		}
		if(Serial2.available()) {
			inByte = Serial2.read();
			Serial1.write(inByte);
		}
	}

	if (x_position != encoderX.getPosition())
	{
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
	  z_position = encoderZ.getPosition();
	  DBGSerial.print("Zpos:");
	  DBGSerial.println(z_position);
	}
	
	if (encoderZ.getPushButton() == true)
	{
		DBGSerial.println(F("Z PRESSED"));
	}

	while(ms.ports[IN_USBSER].available())
	{
		ms.ports[OUT_USBSER].write(ms.ports[0].read());
	}

}
