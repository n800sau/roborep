#include <SPI.h>
#include <SD.h>
#include "STM32LowPower.h"

#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
//#define SERVER_ADDRESS 2

//		Using the second SPI port (SPI_2)
//		SS		<-->	PB12 <-->	BOARD_SPI2_NSS_PIN
//		SCK	 <-->	PB13 <-->	BOARD_SPI2_SCK_PIN
//		MISO	<-->	PB14 <-->	BOARD_SPI2_MISO_PIN
//		MOSI	<-->	PB15 <-->	BOARD_SPI2_MOSI_PIN
//SPIClass SPI_2(PB15, PB14, PB13, PB12); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port

// Singleton instance of the radio driver
// CE, CSN(SS)
RH_NRF24 driver(PB4, PB5);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// for sd
const int chipSelect = PA4;

// 10 sec
#define SLEEP_MS 10000


void setup()
{
	// Open serial communications and wait for port to open:
	Serial.begin(115200);

	Serial.print("Initializing SD card...");

	// see if the card is present and can be initialized:
	if (!SD.begin(chipSelect)) {
		Serial.println("Card failed, or not present");
	} else {
		Serial.println("card initialized.");
	}

	Serial.print("Initializing radio card...");
	if (!manager.init()) {
		Serial.println("radio init failed");
	} else {
		// The default radio config is for 30MHz Xtal, 434MHz base freq 2GFSK 5kbps 10kHz deviation
		// power setting 0x10
		// If you want a different frequency mand or modulation scheme, you must generate a new
		// radio config file as per the RH_RF24 module documentation and recompile
		// You can change a few other things programatically:
		//driver.setFrequency(435.0); // Only within the same frequency band
		//driver.setTxPower(0x7f);


		Serial.println("Radio initialized.");
	}

//	driver.printRegisters();
	Serial.flush();
	LowPower.begin();
}

struct M_DATA {
	uint8_t obj_id;   // unique id (sensor/actuator id)
	uint8_t dev_type; // device type
	uint8_t val_type; // presentation, value, etc
	union {           // value
		int32_t int_val;
		float float_val;
		char str_val[20];
	};
} __attribute__((packed)) buf;

uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];
// Dont put this on the stack:
//uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

void loop()
{
	Serial.println("Sending to nrf24_reliable_datagram_server");
//	strncpy((char*)data, String(random(200, 500)/100., 3).c_str(), sizeof(data));
	memset(&buf, 0, sizeof(buf));
	buf.obj_id = 17;
	buf.val_type = 2;
	buf.float_val = random(200, 500)/100.;
	// Send a message to manager_server
//	if (manager.sendtoWait((uint8_t*)&buf, sizeof(buf), SERVER_ADDRESS))
	if (manager.sendtoWait((uint8_t*)&buf, sizeof(buf), RH_BROADCAST_ADDRESS))
	{
		// Now wait for a reply from the server
		uint8_t len = sizeof(buf);
		uint8_t from;
		if (manager.recvfromAckTimeout((uint8_t*)&buf, &len, 2000, &from))
		{
			Serial.print("got reply from : 0x");
			Serial.print(from, HEX);
			Serial.print(": ");
			Serial.println(buf.obj_id);
			Serial.print(" => ");
			Serial.println(buf.str_val);
		}
		else
		{
			Serial.println("No reply, is nrf24_reliable_datagram_server running?");
		}
	}
	else
		Serial.println("sendtoWait failed");
	driver.sleep();

	// make a string for assembling the data to log:
	String dataString = "";

	// read three sensors and append to the string:
	for (int analogPin = 0; analogPin < 3; analogPin++) {
		int sensor = analogRead(analogPin);
		dataString += String(sensor);
		if (analogPin < 2) {
			dataString += ",";
		}
	}

	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	File dataFile = SD.open("datalog.txt", FILE_WRITE);

	// if the file is available, write to it:
	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close();
		// print to the serial port too:
		Serial.println(dataString);
	}
	// if the file isn't open, pop up an error:
	else {
		Serial.println("error opening datalog.txt");
	}

	Serial.flush();
	LowPower.deepSleep(SLEEP_MS);
}
