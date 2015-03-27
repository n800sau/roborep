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

float stop_acc_x = 0;

int lastLCount = 0, lastRCount = 0;

typedef struct _Vec{
	int lCount, rCount;
	short heading;
} Vec;

#define VECBUF_SIZE 50
Vec vecbuf[VECBUF_SIZE];
int vec_pointer = 0;
bool vec_overflow = false;

void setup()
{
	pinMode(led, OUTPUT);
	digitalWrite(led, 1);
	Serial.begin(115200);
//	Serial.println(F("Starting the I2C interface."));
	Serial.println(F("Reloading..."));
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	setup_irdist();
	setup_presence();
	setup_motors();
	Serial.println(F("Ready"));
}

void printState()
{
	int i;
	int v = readVccMv();
//		Serial.print(vec_pointer);
//		Serial.println(F(" vectors"));
	Serial.print(F("JSON:{\"type\":\"sensors\""));
	Serial.print(F(",\"V\":"));
	Serial.print(v);
	Serial.print(F(",\"LC\":"));
	Serial.print(motor1QeiCounts);
	Serial.print(F(",\"RC\":"));
	Serial.print(motor2QeiCounts);
	Serial.println();

	Serial.print(F(",\"head\":"));
	Serial.print(headingDegrees);
	Serial.print(F(",\"acc_x\":"));
	Serial.print(adxl345_state.event.acceleration.x - stop_acc_x);
	Serial.print(F(",\"acc_x_avg\":"));
	Serial.print(acc_x_avg() - stop_acc_x);
	Serial.print(F(",\"acc_x_max\":"));
	Serial.print(acc_x_max() - stop_acc_x);
	Serial.print(F(",\"acc_hit\":"));
	Serial.print((int)adxl345_state.single_tap);

	adxl345_state.reset();

	Serial.print(F(",\"gyro_x\":"));
	Serial.print((int)gyro.g.x);
	Serial.println();

	Serial.print(F(", \"T\":"));
	float temperature;
	bmp.getTemperature(&temperature);
	Serial.print(temperature);

	Serial.print(F(", \"P\":"));
	Serial.print(bmp085_event.pressure);

	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	Serial.print(F(", \"Alt\":"));
	Serial.print(bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, bmp085_event.pressure));

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
	// Sydney 1018.1 hPa
	Serial.print(F(", \"IRdist\":"));
	Serial.print(distance);
	Serial.print(F(", \"Presence\":"));
	Serial.println(MVcount);

	Serial.print(F(", \"Lcoef\":"));
	Serial.print(motor1Coef);
	Serial.print(F(", \"Rcoef\":"));
	Serial.print(motor2Coef);
	Serial.print(F(", \"vec_over\":"));
	Serial.print(vec_overflow);
	Serial.println(F(", \"vects\": ["));
	int count = min(vec_pointer, 5);
	for(i=0; i<count; i++) {
		if(i > 0) {
			Serial.println(F(","));
		}
		Serial.print(F("{\"lC\":"));
		Serial.print(vecbuf[i].lCount);
		Serial.print(F(",\"rC\":"));
		Serial.print(vecbuf[i].rCount);
		Serial.print(F(",\"h\":"));
		Serial.print(vecbuf[i].heading);
		Serial.print(F("}"));
	}
	Serial.println(F("]"));
	for(i=0; i<VECBUF_SIZE - count; i++) {
		vecbuf[i] = vecbuf[i+count];
	}
	vec_pointer -= count;

	Serial.println(F("}"));
	Serial.println(F("."));
		Serial.println(F("JSON sent"));
}

void ok()
{
	Serial.println(F("{\"reply\":\"ok\"}"));
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
				Serial.println(F("json end"));
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

class Action {
	public:
		virtual bool loop() { return false; }
};

class Spinning: public Action {

	protected:
		int heading, stage;

	public:
		Spinning() {
			turn_left(10000);
			heading = headingDegrees;
			stage = 1;
		}
		virtual bool loop() {
			if(abs(heading-headingDegrees) > 5) {
				Serial.print("{\"dist\":");
				Serial.print(distance);
				Serial.print(",\"head\":");
				Serial.print(headingDegrees);
				Serial.println("}");
				int h = heading;
				if(abs(h - headingDegrees) > 180) {
					h += 360;
				}
				if(abs(h - headingDegrees) > 90) {
					heading = headingDegrees;
					stage++;
				}
				if(stage > 4) {
					stop();
					return false;
				}
			}
			return true;
		}

};

Action *cur_action = NULL;

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
		lastLCount = lastRCount = motor1QeiCounts = motor2QeiCounts = 0;
		ok();
	} else if (strcmp(cmd, "calibrate_motors") == 0) {
		calibrate_motors();
		ok();
	} else if (strcmp(cmd, "set_acc_zero") == 0) {
		stop_acc_x = adxl345_state.event.acceleration.x;
		ok();
	} else if (strcmp(cmd, "spinning") == 0) {
		if(cur_action) {
			delete cur_action;
			stop();
			Serial.println(F("Action switched"));
		}
		cur_action = new Spinning;
		ok();
	}
}

void add_vector()
{
	if(lastLCount != motor1QeiCounts || lastRCount != motor2QeiCounts) {
		vecbuf[vec_pointer].lCount = motor1QeiCounts - lastLCount;
		vecbuf[vec_pointer].rCount = motor2QeiCounts - lastRCount;
		vecbuf[vec_pointer].heading = headingDegrees;
		vec_pointer++;
		if(vec_pointer >= VECBUF_SIZE) {
			for(int i=0; i<VECBUF_SIZE - 1; i++) {
				vecbuf[i] = vecbuf[i+1];
			}
			vec_pointer = VECBUF_SIZE - 1;
			vec_overflow = true;
		}
		lastLCount = motor1QeiCounts;
		lastRCount = motor2QeiCounts;
	}
}

void loop()
{
	unsigned long t = millis();
	if(abs(t - last_time) > 1000) {
		last_time = t;
		digitalWrite(led, lon);
		lon = !lon;
//		Serial.println(F("tick"));
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
			Serial.println(F("JSON parsing failed of"));
			Serial.println(jsonBuf);
		} else {
//			Serial.println(jsonBuf);
			const char* cmd = data["command"];
			Serial.print(F("command:"));
			Serial.println(cmd);
			execute(cmd, data);
		}
	}
	process_motors();
	add_vector();
	if(cur_action) {
		if((!cur_action->loop()) || stopped()) {
			Serial.println(F("Action stopped"));
			delete cur_action;
			cur_action = NULL;
		}
	}
	delay(20);
}
