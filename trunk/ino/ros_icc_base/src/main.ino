#include <voltage.h>
#include "../../include/printf.h"
#include <Wire.h>
#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "motor_proc.h"

// time in millisecs to stop if no encoder reading
#define TIME2STOP 10000

void setup()
{
	Serial.begin(115200);
	printf_begin();

	Serial.println("Starting the I2C interface.");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();

	motor_setup();
}

void printState()
{
	Serial.print((compass_mode) ? "C mode" : "D mode");
	Serial.print(" DATA ");
	Serial.print("lsteps:");
	Serial.print(lDest);
	Serial.print(",rsteps:");
	Serial.print(rDest);
	Serial.print(",lpw:");
	Serial.print(lPower * ((lReverse) ? -1 : 1));
	Serial.print(",rpw:");
	Serial.print(rPower * ((rReverse) ? -1 : 1));
	Serial.print(",lcnt:");
	Serial.print(lCounter);
	Serial.print(",rcnt:");
	Serial.print(rCounter);
//	Serial.print(",x:");
//	Serial.print(x);
//	Serial.print(",y:");
//	Serial.print(y);
//	Serial.print(",fi:");
//	Serial.print(fi);
	Serial.print(",head:");
	Serial.print(headingDegrees);
	Serial.print(",dir:");
	Serial.print(intentDir);
	int intentOffset = -1;
	if(intentDir >= 0) {
		intentOffset = intentDir - headingDegrees;
		if(intentOffset > 180) intentOffset = 360 - intentOffset;
		else if(intentOffset < -180) intentOffset = 360 + intentOffset;
	}
	Serial.print(",off:");
	Serial.print(intentOffset);
	Serial.print(",azim:");
	Serial.print(azimuth);
	Serial.print(",off:");
	Serial.print(offset);
//	Serial.print(",acc_x:");
//	Serial.print(accel_scaled.XAxis);
//	Serial.print(",gyro_x:");
//	Serial.print((int)gyro.g.x);
	Serial.println("");
}

void loop()
{
	unsigned long last_millis = 0;
	unsigned long cur_millis = millis();

	process_compass();
	process_accel();
	process_gyro();

	static int last_lCounter = -1;
	static int last_rCounter = -1;

	unsigned long millisdiff = (last_millis > cur_millis) ? ((unsigned long) -1) - last_millis + cur_millis : cur_millis - last_millis;

	if( millisdiff > TIME2STOP ) {
		stop(true);
	} else {
		if(last_lCounter != lCounter || last_rCounter != rCounter) {
			printState();
			last_lCounter=lCounter;
			last_rCounter=rCounter;
		}
		motor_process();
	}

//		Serial.print(compass_raw.XAxis);
//		Serial.print(":");
//		Serial.print(headingDegrees);
//		Serial.println("");
	char val = Serial.read();
	if(val!=-1)
	{
		Serial.println(val);
		switch(val)
		{
			case '`':
				stop(true);
				compass_mode = !compass_mode;
				break;
			case 'q':
				LstepSize ++;
				Serial.print("left step:");
				Serial.println(LstepSize);
				break;
			case 'z':
				LstepSize --;
				Serial.print("left step:");
				Serial.println(LstepSize);
				break;
			case 'e':
				RstepSize ++;
				Serial.print("right step:");
				Serial.println(RstepSize);
				break;
			case 'c':
				RstepSize --;
				Serial.print("right step:");
				Serial.println(RstepSize);
				break;
			case 'v':
				Serial.print("V:");
				Serial.println(readVccMv());
				break;
			case '0':
				Serial.println("Position reset");
				fi = x = y = 0;
				break;
			case 's'://stop
				stop(true);
				break;
		}
		if(compass_mode) {
			switch(val)
			{
				case 'w'://Move ahead
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = false;
					azimuth = -1;
					break;
				case 'x'://move back
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = true;
					azimuth = -1;
					break;
				case 'y': // go north
					intentDir = 0;
					break;
				case 'b': // go south
					intentDir = 180;
					break;
				case 'g': // go west
					intentDir = 360-90;
					break;
				case 'h': // go east
					intentDir = 90;
					break;
				case 'j': // reduce azimuth
					intentDir = (360 + intentDir - 10) % 360;
					break;
				case 'l': // enlarge azimuth
					intentDir = (360 + intentDir + 10) % 360;
					break;
				case 'k': // start direction
					azimuth = intentDir;
					break;
			}
		} else {
			switch(val)
			{
				case 'w'://Move ahead
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = false;
					break;
				case 'x'://move back
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = true;
					break;
				case 'a'://turn left
					stop();
					lDest = rDest = TstepSize;
					lReverse = true;
					rReverse = false;
					break;
				case 'd'://turn right
					stop();
					lDest = rDest = TstepSize;
					lReverse = false;
					rReverse = true;
					break;
			}
		}
		last_millis = cur_millis;
		printState();
	}
}
