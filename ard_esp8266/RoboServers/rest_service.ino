#include "rest_service.h"
#include "uart_utils.h"
#include <ESP8266WiFi.h>
#include <aREST.h>
#include <Ticker.h>
#include <PString.h>

const int ioResetPin = 4;

#define MESSAGE_SIZE 2000
static String message;

static Ticker unsetter;

// Create aREST instance
static aREST rest;
// Create an instance of the server
static WiFiServer rserver(8080);

static void ICACHE_FLASH_ATTR resetOff()
{
	digitalWrite(ioResetPin, 1);
}

static void ICACHE_FLASH_ATTR doReset()
{
	ESP.reset();
}

static int ICACHE_FLASH_ATTR ioReset(String param)
{
	digitalWrite(ioResetPin, 0);
	unsetter.once(1, resetOff);
	return 0;
}

static int ICACHE_FLASH_ATTR selSer(String param)
{
	uartSetPrimary(param.toInt());
	message = String((uartIsPrimary()) ? "primary": "secondary");
	return 0;
}

static int ICACHE_FLASH_ATTR setBaud(String param)
{
	int baud = param.toInt();
	Serial.begin(baud);
	message = String(baud);
	return 0;
}

static int ICACHE_FLASH_ATTR selfReset(String param)
{
	unsetter.once(1, doReset);
	return 0;
}

static int ICACHE_FLASH_ATTR getVcc(String param)
{
	message = String(ESP.getVcc());
	return 0;
}

static int ICACHE_FLASH_ATTR getFreeHeap(String param)
{
	message = String(ESP.getFreeHeap());
	return 0;
}

static int ICACHE_FLASH_ATTR executeRESTCommand(String command)
{
	int rs = -1;
	static char msgbuf[MESSAGE_SIZE];
	PString pmsg(msgbuf, MESSAGE_SIZE);
	if(command.equalsIgnoreCase("info")) {
		WiFi.printDiag(pmsg);
		message = msgbuf;
		rs = 0;
	}
	return rs;
}

void ICACHE_FLASH_ATTR setupRESTservice()
{
	pinMode(ioResetPin, OUTPUT);
	digitalWrite(ioResetPin, 1);

	// Give name and ID to device
	rest.set_id((char*)String(ESP.getChipId()).c_str());
	rest.set_name((char*)"esp8266");

	rest.function((char*)"ioreset", ioReset);

	rest.function((char*)"reset", selfReset);

	// Function to change serial
	rest.function((char*)"selser", selSer);
	rest.function((char*)"setbaud", setBaud);

	rest.function((char*)"vcc", getVcc);
	rest.function((char*)"free", getFreeHeap);

	// Command function
	rest.function((char*)"command", executeRESTCommand);

	message.reserve(MESSAGE_SIZE);
	rest.variable((char*)"message", &message);

	// Start the REST server
	rserver.begin();
}

void ICACHE_FLASH_ATTR handleRESTservice()
{
	if (rserver.hasClient()) {
		WiFiClient rclient = rserver.available();
		while(!rclient.available()){
			delay(1);
		}
		rest.handle(rclient);
	}
}

