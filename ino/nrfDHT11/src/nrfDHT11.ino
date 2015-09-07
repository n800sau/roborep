// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include <avr/sleep.h>
#include <avr/wdt.h>

#include <DHT.h>
#include <voltage.h>
#include <SPI.h>
#include <RF24.h>

#define DHTPIN A0			// what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11		// DHT 11
//#define DHTTYPE DHT22		// DHT 22	 (AM2302)
//#define DHTTYPE DHT21		// DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.	 This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// nRF24L01(+) radio attached
RF24 radio(8, 9);

byte master_address[6] = "ESPma";
byte my_address[6] = "DHT11";

#define LED_PIN 10

// watchdog interrupt
ISR(WDT_vect) 
{
	wdt_disable();	// disable watchdog
}

void myWatchdogEnable(const byte interval)
{
	Serial.flush();
	delay(200);
	MCUSR = 0;													// reset various flags
	WDTCSR |= 0b00011000;								// see docs, set WDCE, WDE
	WDTCSR =	0b01000000 | interval;		// set WDIE, and appropriate delay

	wdt_reset();
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);
	sleep_mode();						 // now goes to sleep and waits for the interrupt
	delay(200);
} 

void blink(int times=1)
{
	for(int i=0; i<times; i++) {
		digitalWrite(LED_PIN, HIGH);
		delay(100);
		digitalWrite(LED_PIN, LOW);
		if(i < times) {
			delay(200);
		}
	}
}

void setup() {
	Serial.begin(115200);
	Serial.println("DHT nrf24 trasnmitter");
	pinMode(LED_PIN, OUTPUT);
	dht.begin();
	SPI.begin();
	radio.begin();

	// Set the PA Level low to prevent power supply related issues since this is a
	// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	radio.setPALevel(RF24_PA_MAX);

	radio.openWritingPipe(my_address);
	radio.openReadingPipe(1, master_address);

	radio.startListening();
}

void sleep20()
{
	radio.powerDown();

	// sleep for a total of 20 seconds
	myWatchdogEnable (0b100001);	// 8 seconds
	myWatchdogEnable (0b100001);	// 8 seconds
	myWatchdogEnable (0b100000);	// 4 seconds

	// Wait a few seconds between measurements.
//	delay(2000);
}

void loop()
{

	radio.powerUp();

	radio.stopListening();                                    // First, stop listening so we can talk.

	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float h = dht.readHumidity();
	// Read temperature as Celsius (the default)
	float t = dht.readTemperature();

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t)) {
		Serial.println("Failed to read from DHT sensor!");
		blink(5);
	} else {

		// Compute heat index in Celsius (isFahreheit = false)
		float hic = dht.computeHeatIndex(t, h, false);

		int v = readVccMv();

		Serial.print("H: ");
		Serial.print(h);
		Serial.print(" %\t");
		Serial.print("T: ");
		Serial.print(t);
		Serial.print(" *C ");
		Serial.print("Heat: ");
		Serial.print(hic);
		Serial.print(" *C ");
		Serial.print("V:");
		Serial.println(v);


		char buf[32];
		("H" + String(long(h*1000)) + "T" + String(long(t*1000)) + "I" + String(long(hic * 1000)) + "V" + String(v)).toCharArray(buf, sizeof(buf));

//		Serial.print(F("Sending:"));
//		Serial.println(buf);

		if (!radio.write(buf, sizeof(buf))){
			Serial.println(F("Radio failed"));
			blink(4);
		} else {
			Serial.println(F("Written"));

			radio.startListening();									// Now, continue listening

			unsigned long started_waiting_at = micros();			   // Set up a timeout period, get the current microseconds
			boolean timeout = false;								   // Set up a variable to indicate if a response was received or not

			while ( ! radio.available() ){							 // While nothing is received
				if (micros() - started_waiting_at > 200000 ){			// If waited longer than 200ms, indicate timeout and exit while loop
					timeout = true;
					break;
				}
			}

			if ( timeout ){											 // Describe the results
				Serial.println(F("Failed, response timed out."));
				blink(3);
			}else{
				char buf[3];
				radio.read(buf, sizeof(buf));

				Serial.print(F("Got response:"));
				Serial.println(buf);
				blink();
			}
		}
	}

	sleep20();
}
