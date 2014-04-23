#include <avr/sleep.h>

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
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high




// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


enum CORR_TYPE {AX=0, AY, AZ, GX, GY, GZ};
int16_t corrs[] = {-1660 /* G */, 750, -550, 0, 0, 0};
//int16_t corrs[] = {-5130, 1320, -1017, -141, 55, 86};
int16_t vals[6];

int wakeInt = 1;                 // pin used for waking up
volatile bool interrupted;

void setup() {


  pinMode(wakeInt, INPUT);

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
	mpu.reset();
	delay(200);
	mpu.initialize();

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// use the code below to change accel/gyro offset values

	Serial.println("Updating internal sensor offsets...");
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
	mpu.setXAccelOffset(corrs[0]);
	mpu.setYAccelOffset(corrs[1]);
	mpu.setZAccelOffset(corrs[2]);
	mpu.setXGyroOffset(corrs[3]);
	mpu.setYGyroOffset(corrs[4]);
	mpu.setZGyroOffset(corrs[5]);
	Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); // -76
	Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); // -2359
	Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); // 1688
	Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 0
	Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 0
	Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 0
	Serial.println();

	mpu.setInterruptMode(MPU6050_INTMODE_ACTIVELOW);
//	mpu.setIntEnabled((1 << MPU6050_INTERRUPT_MOT_BIT) | (1 << MPU6050_INTERRUPT_ZMOT_BIT));
	mpu.setIntEnabled((1 << MPU6050_INTERRUPT_MOT_BIT) | (1 << MPU6050_INTERRUPT_DATA_RDY_BIT));

	mpu.setMotionDetectionThreshold(1);
	mpu.setMotionDetectionDuration(4);

	mpu.setZeroMotionDetectionThreshold(156);
	mpu.setZeroMotionDetectionDuration(0);

	mpu.setClockSource(MPU6050_CLOCK_INTERNAL);
	mpu.setWakeFrequency(MPU6050_WAKE_FREQ_10);
	mpu.setWakeCycleEnabled(true);

//    attachInterrupt(wakeInt, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function

}

void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
	interrupted = true;
}

void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we 
     * choose the according 
     * sleep mode: SLEEP_MODE_PWR_DOWN
     * 
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    /* Now it is time to enable an interrupt. We do it here so an 
     * accidentally pushed interrupt button doesn't interrupt 
     * our running program. if you want to be able to run 
     * interrupt code besides the sleep function, place it in 
     * setup() for example.
     * 
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
     * 
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */

    attachInterrupt(wakeInt, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW 

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(wakeInt);      // disables interrupt 0 on pin 2 so the 
                             // wakeUpNow code will not be executed 
                             // during normal running time.

}

void loop() {

	delay(1000);                           // waits for a second

	if(interrupted) {

		interrupted = false;

		int st = mpu.getIntStatus();
		Serial.print("\tst: 0x");
		Serial.println(st, HEX);
		int i;

		// read raw accel/gyro measurements from device
		mpu.getMotion6(&vals[AX], &vals[AY], &vals[AZ], &vals[GX], &vals[GY], &vals[GZ]);


		if(st & (1 << MPU6050_INTERRUPT_ZMOT_BIT)) {
			Serial.println("Stopped");
		}

		if(st & (1 << MPU6050_INTERRUPT_MOT_BIT)) {
			Serial.print("Moving:");
			Serial.print(mpu.getXNegMotionDetected());
			Serial.print("-X-");
			Serial.print(mpu.getXPosMotionDetected());
			Serial.print(" ");
			Serial.print(mpu.getYNegMotionDetected());
			Serial.print("-Y-");
			Serial.print(mpu.getYPosMotionDetected());
			Serial.print(" ");
			Serial.print(mpu.getZNegMotionDetected());
			Serial.print("-Z-");
			Serial.print(mpu.getZPosMotionDetected());
			Serial.println();
		}


		// these methods (and a few others) are also available
		//mpu.getAcceleration(&ax, &ay, &az);
		//mpu.getRotation(&gx, &gy, &gz);

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
	}

	delay(100);     // this delay is needed, the sleep 
                      //function will provoke a Serial error otherwise!!
	sleepNow();     // sleep function called here

}
