#include <SPI.h>
#include <RF24_STM32.h>

#include <STM32Sleep.h>
#include <RTClock.h>

#include <Wire.h>
#include <SparkFunCCS811.h>

#include <Adafruit_GFX_AS.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library

#include <EEPROM.h>

#include <AM2320.h>


#define TFT_CS     PA4
#define TFT_RST    PA2
#define TFT_DC     PA3
#define TFT_SCLK   PA5   // set these to be whatever pins you like!
#define TFT_MOSI   PA7   // set these to be whatever pins you like!
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

SPIClass SPI_1(2);

RTClock rt(RTCSEL_LSE);

//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address

CCS811 ccs(CCS811_ADDR);

AM2320 am2320;

#define CCS_WAKE_PIN  PB5
#define CCS_INT_PIN  PB4
#define CCS_RESET_PIN  PB3

#define LED_PIN PC13

#define INT_BUTTON_PIN PB0

#define EXT_BUTTON1_PIN PA11
#define EXT_BUTTON2_PIN PA12
#define EXT_BUTTON3_PIN PA15
#define EXT_BUTTON4_PIN PB9

#define TFT_PIN PB8

bool ccs_ready = true;

#define EE_OFFSET 3000

/****************** User Config ***************************/
// ce, cs
RF24 radio(PA8, PB12);
/**********************************************************/

byte addresses[][5] = {{0xc2, 0xc2, 0xc2, 0xc2, 0xc2},{0xe7, 0xe7, 0xe7, 0xe7, 0xe7}};

void led_on()
{
	digitalWrite(LED_PIN, LOW);
}

void led_off()
{
	digitalWrite(LED_PIN, HIGH);
}

bool has_data = false;
int co2, tvoc;
float d_temp;
float d_hum;
time_t ccs_ts, d_ts;
// time difference
int ccs_dt, d_dt;
unsigned int currentBaseLine = 0;

void blink(int times=1)
{
	for(int i=0; i<times; i++) {
		led_on();
		delay(100);
		led_off();
		if(i < times) {
			delay(200);
		}
	}
}

CCS811Core::status printCCSError(CCS811Core::status errorCode)
{
	switch ( errorCode )
	{
		case CCS811Core::SENSOR_SUCCESS:
			break;
		case CCS811Core::SENSOR_ID_ERROR:
			Serial.print("ID_ERROR");
			break;
		case CCS811Core::SENSOR_I2C_ERROR:
			Serial.print("I2C_ERROR");
			break;
		case CCS811Core::SENSOR_INTERNAL_ERROR:
			Serial.print("INTERNAL_ERROR");
			break;
		case CCS811Core::SENSOR_GENERIC_ERROR:
			Serial.print("GENERIC_ERROR");
			break;
		default:
			Serial.print("Unspecified error.");
	}
	return errorCode;
}

//printSensorError gets, clears, then prints the errors
//saved within the error register.
void printSensorError()
{
	uint8_t error = ccs.getErrorRegister();

	if ( error == 0xFF ) //comm error
	{
		Serial.println("Failed to get ERROR_ID register.");
	}
	else
	{
		Serial.print("Error: ");
		if (error & 1 << 5) Serial.print("HeaterSupply");
		if (error & 1 << 4) Serial.print("HeaterFault");
		if (error & 1 << 3) Serial.print("MaxResistance");
		if (error & 1 << 2) Serial.print("MeasModeInvalid");
		if (error & 1 << 1) Serial.print("ReadRegInvalid");
		if (error & 1 << 0) Serial.print("MsgInvalid");
		Serial.println();
	}
}

// bindex:
// 0 - internal
// 1-4 external
bool is_button_pressed(int bindex)
{
	bool rs = false;
	switch(bindex) {
		case 0:
			rs = !digitalRead(INT_BUTTON_PIN);
			break;
		case 1:
			rs = !digitalRead(EXT_BUTTON1_PIN);
			break;
		case 2:
			rs = !digitalRead(EXT_BUTTON2_PIN);
			break;
		case 3:
			rs = !digitalRead(EXT_BUTTON3_PIN);
			break;
		case 4:
			rs = !digitalRead(EXT_BUTTON4_PIN);
			break;
	}
	return rs;
}

void setup()
{
	adc_disable_all();
	setGPIOModeToAllPins(GPIO_INPUT_ANALOG);

	Serial.begin(115200);
	Serial.println("Started");

	pinMode(INT_BUTTON_PIN, INPUT_PULLUP);
	pinMode(EXT_BUTTON1_PIN, INPUT_PULLUP);
	pinMode(EXT_BUTTON2_PIN, INPUT_PULLUP);
	pinMode(EXT_BUTTON3_PIN, INPUT_PULLUP);
	pinMode(EXT_BUTTON4_PIN, INPUT_PULLUP);

	pinMode(LED_PIN, OUTPUT);
	led_on();
	pinMode(TFT_PIN, OUTPUT);
	digitalWrite(TFT_PIN, HIGH);

	am2320.begin();

	pinMode(CCS_WAKE_PIN, OUTPUT);   // set WAKE pin as OUTPUT
	digitalWrite(CCS_WAKE_PIN, LOW);

	pinMode(CCS_RESET_PIN, OUTPUT);   // set WAKE pin as OUTPUT
	digitalWrite(CCS_RESET_PIN, LOW);
	delay(500);
	digitalWrite(CCS_RESET_PIN, HIGH);

	// Use this initializer if you're using a 1.8" TFT
	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
	tft.setTextWrap(false); // Allow text to run off right edge
	tft.fillScreen(ST7735_BLACK);
	tft.setRotation(1);

	radio.begin();
	radio.setChannel(0x4c);
	radio.setPALevel(RF24_PA_HIGH);
	radio.setDataRate(RF24_1MBPS);
	radio.setAutoAck(true);
	radio.setRetries(10, 20);
	radio.setCRCLength(RF24_CRC_16);
	radio.enableDynamicPayloads();

	radio.openWritingPipe(addresses[0]);
	radio.openReadingPipe(1, addresses[1]);

//	radio.startListening();
	radio.stopListening();
	radio.printDetails();
	radio.powerDown();

	ccs_ready = printCCSError(ccs.begin()) == CCS811Core::SENSOR_SUCCESS;

	//This looks for previously saved data in the eeprom at program start
	if ((EEPROM.read(EE_OFFSET) == 0xA5) && (EEPROM.read(EE_OFFSET+1) == 0xB2))
	{
		Serial.println("EEPROM contains saved data.");
		// load baseline if button is not pressed
		if(is_button_pressed(2)) {
			//The recovered baseline is packed into a 16 bit word
			currentBaseLine = ((unsigned int)EEPROM.read(EE_OFFSET+2) << 8) | EEPROM.read(EE_OFFSET+3);
			Serial.print("Saved baseline: 0x");
			if (currentBaseLine < 0x100) Serial.print("0");
			if (currentBaseLine < 0x10) Serial.print("0");
			Serial.println(currentBaseLine, HEX);
			//This programs the baseline into the sensor and monitors error states
			printCCSError(ccs.setBaseline(currentBaseLine));
			blink(2);
		} else {
			blink(5);
		}
	} else {
		Serial.println("Saved data not found!");
	}

	digitalWrite(CCS_WAKE_PIN, HIGH);

	ccs_ts = d_ts = rt.getTime();
}

bool send_line(const char buf[])
{
	bool rs;
	radio.powerUp();
//	rs = radio.writeBlocking(buf, strlen(buf)+1, 1000);
	rs = radio.writeFast(buf, strlen(buf)+1);
	// to be sure that transfer is successful before powerdown
	rs = radio.txStandBy(1000, true);
	if(rs){
		Serial.print(F("Sent "));
		Serial.println(buf);
	} else {
		Serial.println(F("Send failed"));
	}
	radio.powerDown();
	return rs;
}

bool send_data()
{
	char buf[32];
	snprintf(buf, sizeof(buf), "%d,%d", co2, tvoc);
	return send_line(buf);
}

void collect_data()
{
	digitalWrite(CCS_WAKE_PIN, LOW);
	for(int i=0; i<10; i++) {
		if(am2320.measure()) {
			d_temp = am2320.getTemperature();
			d_hum = am2320.getHumidity();
			time_t t = rt.getTime();
			d_dt = t - d_ts;
			d_ts = t;
			printCCSError(ccs.setEnvironmentalData(d_hum, d_temp));
			Serial.print("T: ");
			Serial.print(d_temp);
			Serial.print(", H: ");
			Serial.print(d_hum);
			Serial.print(", d_dt: ");
			Serial.println(d_dt);
			break;
		}  else {  // error has occured
			int errorCode = am2320.getErrorCode();
			switch (errorCode) {
//				case 1: Serial.println("ERR: Sensor is offline"); break;
				case 2: Serial.println("ERR: CRC validation failed."); break;
			}
			delay(500);
		}
	}
	delay(2000);
	if(ccs.dataAvailable()){
		printCCSError(ccs.readAlgorithmResults());
		co2 = ccs.getCO2();
		tvoc = ccs.getTVOC();
		Serial.print("CO2: ");
		Serial.print(co2);
		Serial.print("ppm, TVOC: ");
		Serial.print(tvoc);
		Serial.print("ppb");
		Serial.print(", dt: ");
		Serial.println(ccs_dt);
		time_t t = rt.getTime();
		ccs_dt = t - ccs_ts;
		ccs_ts = t;
		has_data = true;
		// save baseline if button is pressed
		if(is_button_pressed(0)) {
			//This gets the latest baseline from the sensor
			currentBaseLine = ccs.getBaseline();
			Serial.print("baseline for this sensor: 0x");
			if (currentBaseLine < 0x100) Serial.print("0");
			if (currentBaseLine < 0x10) Serial.print("0");
			Serial.println(currentBaseLine, HEX);
			//The baseline is saved (with valid data indicator bytes)
			EEPROM.write(EE_OFFSET, 0xA5);
			EEPROM.write(EE_OFFSET+1, 0xB2);
			EEPROM.write(EE_OFFSET+2, (currentBaseLine >> 8) & 0x00FF);
			EEPROM.write(EE_OFFSET+3, currentBaseLine & 0x00FF);
			blink(2);
		}
	} else if (ccs.checkForStatusError()) {
		printSensorError();
	} else {
		Serial.println("CCS data not available!");
	}
	digitalWrite(CCS_WAKE_PIN, HIGH);
}

void turn_tft_on()
{
	digitalWrite(TFT_PIN, HIGH);
}

void turn_tft_off()
{
	digitalWrite(TFT_PIN, LOW);
}

void tft_show_data()
{
	tft.setCursor(0, 30);
	tft.setTextColor(ST7735_YELLOW);
	tft.setTextSize(1);
	tft.fillScreen(ST7735_BLACK);
	tft.print("CO2: ");
	tft.print(co2);
	tft.println("ppm");
	tft.print("TVOC: ");
	tft.print(tvoc);
	tft.println("ppb");
	tft.print("CCS dt:");
	tft.println(ccs_dt);
	tft.print("D Temp:");
	tft.println((int)d_temp);
	tft.print("D Hum:");
	tft.println(d_hum);
	tft.print("D dt:");
	tft.println(d_dt);
	if(currentBaseLine) {
		tft.print("saved baseline:");
		tft.println(currentBaseLine, HEX);
	}
	tft.print("buttons:");
	for(int i=0; i<5; i++) {
		tft.print(is_button_pressed(i) ? "1 " : "0 ");
	}
	tft.println();
}

void loop()
{
	if(ccs_ready) {
		collect_data();
	} else {
		Serial.println("Not ready");
	}

	if(has_data) {
		if(send_data()) {
			has_data = false;
		}
	}

	if(is_button_pressed(1)) {
		turn_tft_on();
	} else {
		turn_tft_off();
	}
	tft_show_data();

	Serial.flush();
	radio.powerDown();
	led_off();
	sleepAndWakeUp(STOP, &rt, 2);
	// recover frequency
	switchToPLLwithHSE(RCC_PLLMUL_9);
	led_on();
}

