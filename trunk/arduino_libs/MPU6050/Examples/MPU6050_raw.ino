// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high




// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

enum CORR_TYPE {AX=0, AY, AZ, GX, GY, GZ};
int16_t corrs[] = {-5130, 1320, -1017, -141, 55, 86};
int16_t vals[6];

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(57600);

	// initialize device
	Serial.println("Initializing I2C devices...");
	accelgyro.initialize();

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// use the code below to change accel/gyro offset values

	Serial.println("Updating internal sensor offsets...");
	accelgyro.setXAccelOffset(corrs[0]);
	accelgyro.setYAccelOffset(corrs[1]);
	accelgyro.setZAccelOffset(corrs[2]);
	accelgyro.setXGyroOffset(corrs[3]);
	accelgyro.setYGyroOffset(corrs[4]);
	accelgyro.setZGyroOffset(corrs[5]);
	Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
	Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
	Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
	Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
	Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
	Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
	Serial.println();


	// configure Arduino LED for
	pinMode(LED_PIN, OUTPUT);
}

void loop() {

	int i;

	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&vals[AX], &vals[AY], &vals[AZ], &vals[GX], &vals[GY], &vals[GZ]);


	// these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);

	#ifdef OUTPUT_READABLE_ACCELGYRO
		// display tab-separated accel/gyro x/y/z values
		Serial.print("a/g:\t");
		for(i=0; i<6; i++) {
			Serial.print(vals[i]); Serial.print("\t");
		}
		Serial.println();
	#endif

	#ifdef OUTPUT_BINARY_ACCELGYRO
		for(i=0; i<6; i++) {
			Serial.write((uint8_t)(vals[i] >> 8)); Serial.write((uint8_t)(vals[i] & 0xFF));
		}
		Serial.println();
	#endif



	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(LED_PIN, blinkState);
}
