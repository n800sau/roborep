/*
  NTP TZ DST - bare minimum
  NetWork Time Protocol - Time Zone - Daylight Saving Time

  Our target for this MINI sketch is:
  - get the SNTP request running
  - set the timezone
  - (implicit) respect daylight saving time
  - how to "read" time to be printed to Serial.Monitor
  
  This example is a stripped down version of the NTP-TZ-DST (v2)
  And works for ESP8266 core 2.7.4 and 3.0.2

  by noiasca
  2020-09-22
*/

#ifndef STASSID
#define STASSID "TVSlow Internet Connection"														// set your SSID
#define STAPSK "1,tpGfhjkz2"												// set your wifi password
#endif

/* Configuration of NTP */
#define MY_NTP_SERVER "at.pool.ntp.org"					 
#define MY_TZ "AEST-10AEDT,M10.1.0,M4.1.0/3"	 

/* Necessary Includes */
#include <ESP8266WiFi.h>						// we need wifi to get internet access
#include <time.h>									 // time() ctime()

//We always have to include the library
#include <LedControl_SW_SPI.h>

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn 
 pin 11 is connected to the CLK 
 pin 10 is connected to LOAD 
 We have only a single MAX72XX.
 */
LedControl_SW_SPI lc;

/* we always wait a bit between updates of the display */
unsigned long delaytime=250;

time_t now;												 // this is the epoch
tm tm;															// the structure tm holds time information in a more convient way

void showTime()
{
	time(&now);											 // read the current time
	localtime_r(&now, &tm);					 // update the structure tm with the current time
	Serial.print("year:");
	Serial.print(tm.tm_year + 1900);	// years since 1900
	Serial.print("\tmonth:");
	Serial.print(tm.tm_mon + 1);			// January = 0 (!)
	Serial.print("\tday:");
	Serial.print(tm.tm_mday);				 // day of month
	Serial.print("\thour:");
	Serial.print(tm.tm_hour);				 // hours since midnight	0-23
	Serial.print("\tmin:");
	Serial.print(tm.tm_min);					// minutes after the hour	0-59
	Serial.print("\tsec:");
	Serial.print(tm.tm_sec);					// seconds after the minute	0-61*
	Serial.print("\twday");
	Serial.print(tm.tm_wday);				 // days since Sunday 0-6
	if (tm.tm_isdst == 1)						 // Daylight Saving Time flag
		Serial.print("\tDST");
	else
		Serial.print("\tstandard");
	Serial.println();
}

void setup()
{
	/*
	 The MAX72XX is in power-saving mode on startup,
	 we have to do a wakeup call
	 */
	lc.begin(D7,D5,D6,1);


	lc.shutdown(0,false);
	/* Set the brightness to a medium values */
	lc.setIntensity(0,1);
	/* and clear the display */
	lc.clearDisplay(0);

	Serial.begin(115200);
	Serial.println("\nNTP TZ DST - bare minimum");

	configTime(MY_TZ, MY_NTP_SERVER); // --> Here is the IMPORTANT ONE LINER needed in your sketch!

	// start network
	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	WiFi.begin(STASSID, STAPSK);
	while (WiFi.status() != WL_CONNECTED) {
		delay(200);
		Serial.print ( "." );
		show_process('_');
	}
	Serial.println("\nWiFi connected");
	// by default, the NTP will be started after 60 secs
	const int EPOCH_1_1_2019 = 1546300800;
 while (time(nullptr) < EPOCH_1_1_2019)
	{
		delay(500);
		Serial.print("*");
		show_process('-');
	}

}

void loop()
{
	showTime();
	writeTimeToSegments();
	delay(100); // dirty delay
}

void writeTimeToSegments()
{
	time(&now);											 // read the current time
	localtime_r(&now, &tm);					 // update the structure tm with the current time
	Serial.println();
//		lc.clearDisplay(0);
//		delay(delaytime);
		// print the hour, minute and second:
	lc.setDigit(0, 7, tm.tm_hour / 10, false);
	lc.setDigit(0, 6, tm.tm_hour % 10, false);
	lc.setChar(0,5,'-', false);
	lc.setDigit(0, 4, tm.tm_min / 10, false);
	lc.setDigit(0, 3, tm.tm_min % 10, false);
	lc.setChar(0,2,'-', false);
	lc.setDigit(0, 1, tm.tm_sec / 10, false);
	lc.setDigit(0, 0, tm.tm_sec % 10, false);
	delay(delaytime);
}

void show_process(char sym)
{
	static int pos = 0;
	lc.clearDisplay(0);
	lc.setChar(0, pos++,sym, false);
	delay(delaytime);
	if(pos > 7) pos = 0;
}
