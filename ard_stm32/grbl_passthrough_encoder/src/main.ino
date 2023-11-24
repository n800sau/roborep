#include <RotaryEncoder.h>

int16_t x_position = 0, y_position = 0, z_position=0;

// p1, p2, button
const int xp1=PB12, xp2=PB14, xb=PB15;
RotaryEncoder encoderX(xp1, xp2, xb);

const int yp1=PB10, yp2=PB9, yb=PB8;
RotaryEncoder encoderY(yp1, yp2, yb);

const int zp1=PA4, zp2=PA5, zb=PA6;
RotaryEncoder encoderZ(zp1, zp2, zb);

// hw-621
#define LED_PIN PB13

// rx3,tx3 - pb11,pb10
// rx2,tx2 - pa3,pba2
// rx1,tx1 - pa10,pa9

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
	encoderX.begin();
	encoderY.begin();
	encoderZ.begin();

	attachInterrupt(digitalPinToInterrupt(xp1), XencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(xb), XencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(yp1), YencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(yb), YencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(zp1), ZencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(zb), ZencoderButtonISR, FALLING);

	Serial.begin(115200);
	Serial1.begin(115200);
	Serial2.begin(115200);
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
	  Serial.print("Xpos:");
	  Serial.println(x_position);
	}
	
	if (encoderX.getPushButton() == true) {
		Serial.println(F("X PRESSED"));
	}

	if (y_position != encoderY.getPosition())
	{
	  y_position = encoderY.getPosition();
	  Serial.print("Ypos:");
	  Serial.println(y_position);
	}
	
	if (encoderY.getPushButton() == true) {
		Serial.println(F("Y PRESSED"));
	}

	if (z_position != encoderZ.getPosition())
	{
	  z_position = encoderZ.getPosition();
	  Serial.print("Zpos:");
	  Serial.println(z_position);
	}
	
	if (encoderZ.getPushButton() == true) {
		Serial.println(F("Z PRESSED"));
	}

}
