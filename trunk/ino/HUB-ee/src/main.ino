#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"
#include "motor_proc.h"
#include "irdist_proc.h"
#include "presence_proc.h"
#include <voltage.h>
#include <ArduinoJson.h>

// hmc5883l - 0x1e
// adxl345 - 0x53
// bmp085 - 0x77
// l3g4200d - 0x69


int led = 6;
int lon = 0;
unsigned long last_time = millis();


void setup()
{
	pinMode(led, OUTPUT);
	digitalWrite(led, 1);
	Serial.begin(115200);
//	Serial.println("Starting the I2C interface.");
	Serial.println("Reloading...");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	setup_irdist();
	setup_presence();
	setup_motors();
	Serial.println("Ready");
}

void printState()
{
	int v = readVccMv();
	Serial.print("JSON:{\"type\":\"sensors\"");
	Serial.print(",\"V\":");
	Serial.print(v);
	Serial.print(",\"LC\":");
	Serial.print(motor1QeiCounts);
	Serial.print(",\"RC\":");
	Serial.print(motor2QeiCounts);
	Serial.println();

	Serial.print(",\"head\":");
	Serial.print(headingDegrees);
	Serial.print(",\"acc_x\":");
	Serial.print(accel_scaled.XAxis);
	Serial.print(",\"gyro_x\":");
	Serial.print((int)gyro.g.x);
	Serial.println();

	Serial.print(", \"T\":");
	Serial.print(bmp.readTemperature());

	Serial.print(", \"P\":");
	Serial.print(bmp.readPressure());

	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	Serial.print(", \"Alt\":");
	Serial.print(bmp.readAltitude());

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
	// Sydney 1018.1 hPa
	Serial.print(", \"R.alt\":");
	Serial.print(bmp.readAltitude(101500));
	Serial.print(", \"IRdist\":");
	Serial.print(distance);
	Serial.print(", \"Presence\":");
	Serial.println(MVcount);

	Serial.print(", \"Lcoef\":");
	Serial.print(motor1Coef);
	Serial.print(", \"Rcoef\":");
	Serial.print(motor2Coef);

	Serial.println("}");
	Serial.println(".");
}

void ok()
{
	Serial.println("{\"reply\":\"ok\"}");
}

static char jsonBuf[256];
static int inputPos = 0;
static boolean jsonComplete = false;

void serialEvent() {
	while (Serial.available() && !jsonComplete) {
		char inChar = (char)Serial.read();
//		Serial.println((int)inChar);
		// change 0xd to 0xa
		if( inChar == 0xd ) inChar = 0xa;
		// skip double 0xa or the first 0xa
		if( ! (inChar == 0xa && (inputPos <= 0 || jsonBuf[inputPos-1] == 0xa)) ) {
			jsonBuf[inputPos] = inChar;
			jsonBuf[++inputPos] = 0;
			// check for the end of json
			if((inputPos >= sizeof(jsonBuf) - 1) ||
					(inputPos >= 2 &&
					(inputPos < 3 || jsonBuf[inputPos-3] == 0xa) &&
					(jsonBuf[inputPos-2] == '.') &&
					(jsonBuf[inputPos-1] == 0xa))
			) {
				// end of json
				Serial.println("json end");
				// check for empty json
				if(inputPos >= 5) {
					// not empty
					// remove useless tail
					jsonBuf[inputPos-3] = 0;
					jsonComplete = true;
				}
				inputPos = 0;
			}
		}
	}
}

void execute(const char *cmd, JsonObject &data)
{
	if(strcmp(cmd, "sensors") == 0) {
		printState();
	} else if (strcmp(cmd, "step_forward") == 0) {
		mv_forward(1000);
		ok();
	} else if (strcmp(cmd, "step_back") == 0) {
		mv_back(1000);
		ok();
	} else if (strcmp(cmd, "turn_left") == 0) {
		turn_left(1000);
		ok();
	} else if (strcmp(cmd, "turn_right") == 0) {
		turn_right(1000);
		ok();
	} else if (strcmp(cmd, "reset_encoders") == 0) {
		motor1QeiCounts = motor2QeiCounts = 0;
		ok();
	} else if (strcmp(cmd, "calibrate_motors") == 0) {
		calibrate_motors();
		ok();
	}
}

void loop()
{
	unsigned long t = millis();
	if(abs(t - last_time) > 1000) {
		last_time = t;
		digitalWrite(led, lon);
		lon = !lon;
		Serial.println("tick");
	}
	process_compass();
	process_accel();
	process_gyro();
	process_bmp085();
	process_irdist();
	process_presence();

	if (jsonComplete) {
		jsonComplete = false;
		StaticJsonBuffer<32> parser;
		JsonObject &data = parser.parseObject(jsonBuf);
		if (!data.success())
		{
			Serial.println("JSON parsing failed of");
			Serial.println(jsonBuf);
		} else {
//			Serial.println(jsonBuf);
			const char* cmd = data["command"];
			Serial.print("command:");
			Serial.println(cmd);
			execute(cmd, data);
		}
	}
	process_motors();
	delay(20);
}
