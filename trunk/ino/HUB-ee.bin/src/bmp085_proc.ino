#include "bmp085_proc.h"

sensors_event_t bmp085_event;

Adafruit_BMP085_Unified bmp;

void setup_bmp085()
{
	bmp = Adafruit_BMP085_Unified(10085);
	if(!bmp.begin())
	{
		Serial.println(F("Could not connect to BMP085."));
	}
}

void process_bmp085()
{
  bmp.getEvent(&bmp085_event);
}

