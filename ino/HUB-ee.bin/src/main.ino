#include "a_spinning.h"
#include "a_turntoheading.h"
#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"
#include "motor_proc.h"
#include "irdist_proc.h"
#include "presence_proc.h"
#include <voltage.h>
#include <crc.h>
#include "commands.h"

// hmc5883l - 0x1e
// adxl345 - 0x53
// bmp085 - 0x77
// l3g4200d - 0x69

// A1 - GPIO13

int led = 6;
int lon = 0;
unsigned long last_time = millis();

float stop_acc_x = 0;
float stop_acc_y = 0;
float stop_acc_z = 0;

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

void sendFloats(byte cmd, float vals[], int count)
{
	if( count <= 255 ) {
		byte bbuf[3 + sizeof(float) * count + 1];
		bbuf[0] = MAGIC_BYTE;
		bbuf[1] = cmd;
		bbuf[2] = sizeof(float) * count;
		for(int i=0; i<count; i++) {
			((float*)(bbuf+3))[i] = vals[i];
		}
		bbuf[3+bbuf[2]] = crc8(bbuf, 3+bbuf[2]);
		Serial.write(bbuf, 4+bbuf[2]);
	} else {
		Serial.println("Array size is bigger than 255");
	}
}

void sendState()
{
	float vals[3] = {
		adxl345_state.event.acceleration.x - stop_acc_x,
		adxl345_state.event.acceleration.y - stop_acc_y,
		adxl345_state.event.acceleration.z - stop_acc_z
	};
	sendFloats(R_ACC_3F, vals, 3);
	vals[0] = readVccMv();
	sendFloats(R_VOLTS_1F, vals, 1);
	Serial.println("Send VO");
	vals[0] = motor1QeiCounts;
	vals[1] = motor2QeiCounts;
	sendFloats(R_MCOUNTS_2F, vals, 2);
	vals[0] = headingDegrees;
	sendFloats(R_HEADING_1F, vals, 1);
	vals[0] = acc_x_avg() - stop_acc_x;
	vals[1] = acc_y_avg() - stop_acc_y;
	vals[2] = acc_z_avg() - stop_acc_z;
	sendFloats(R_ACCAVG_3F, vals, 3);
	vals[0] = acc_x_max() - stop_acc_x;
	vals[1] = acc_y_max() - stop_acc_y;
	vals[2] = acc_z_max() - stop_acc_z;
	sendFloats(R_ACCMAX_3F, vals, 3);
	vals[0] = adxl345_state.single_tap;
	sendFloats(R_HIT_1F, vals, 1);
	vals[0] = gyro.g.x;
	vals[1] = gyro.g.y;
	vals[2] = gyro.g.z;
	sendFloats(R_GYRO_3F, vals, 3);
	bmp.getTemperature(vals);
	sendFloats(R_TEMPERATURE_1F, vals, 1);
	vals[0] = bmp085_event.pressure;
	sendFloats(R_PRESSURE_1F, vals, 1);
	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	// you can get a more precise measurement of altitude
	// if you know the current sea level pressure which will
	// vary with weather and such. If it is 1015 millibars
	// that is equal to 101500 Pascals.
	// Sydney 1018.1 hPa
	vals[0] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, bmp085_event.pressure);
	sendFloats(R_ALT_1F, vals, 1);
	vals[0] = distance;
	sendFloats(R_DISTANCE_1F, vals, 1);
	vals[0] = MVcount;
	sendFloats(R_PR_1F, vals, 1);
	vals[0] = motor1Coef;
	vals[1] = motor2Coef;
	sendFloats(R_MCOEF_2F, vals, 2);
	vals[0] = vec_overflow;
	sendFloats(R_VEC_OVERFLOW_1F, vals, 1);
	sendFloats(R_END, vals, 0);

	int i;
	for(i=0; i<vec_pointer; i++) {
		vals[0] = vecbuf[i].lCount;
		vals[1] = vecbuf[i].rCount;
		vals[2] = vecbuf[i].heading;
		sendFloats(R_VECTOR_3F, vals, 3);
	}
	vec_pointer = 0;

	adxl345_state.reset();

}

void ok()
{
	Serial.println(F("{\"reply\":\"ok\"}"));
}


// format
// 1b magic_byte(0x85)
// 1b command
// 1b data size without crc
// <size>b data
// 1b crc of 

static int inputPos = -1;
// data
static byte reqBuf[266]; // command,datasize,data
static byte *command = reqBuf;
static byte *datasize = reqBuf+1;
static byte *dataBuf = reqBuf+2;

static boolean reqComplete = false;

void resetInput()
{
	// too much of bytes
	// cancel it
	inputPos = -1;
	reqComplete = false;
}

void serialEvent() {
	while (Serial.available() && !reqComplete) {
		byte inChar = Serial.read();
		if( inputPos < 0 && inChar == MAGIC_BYTE ) {
//	Serial.println("New start");
			inputPos = 0;
		} else {
			if(inputPos >= 0) {
				reqBuf[inputPos++] = inChar;
				if(inputPos >= sizeof(reqBuf)) {
					Serial.println(F("Too much bytes in the request"));
					inputPos = -1;
				} else {
//	Serial.print("In:");
//	Serial.println(inChar, HEX);
					if(inputPos > 2) {
						if(inputPos - 3 >= *datasize) {
//		Serial.print("CMP:");
//		Serial.print(inChar, HEX);
//		Serial.print(" with CRC:");
//		Serial.println(crc8(reqBuf, inputPos-1), HEX);
							if(inChar != crc8(reqBuf, inputPos-1)) {
								Serial.println(F("CRC error"));
							} else {
								reqComplete = true;
							}
							inputPos = -1;
						}
					}
				}
			}
		}
	}
}

Action *cur_action = NULL;
const char *action_switched_msg = "Action switched";

void start_action(Action *action)
{
	if(cur_action) {
		delete cur_action;
		stop();
		Serial.println(action_switched_msg);
	}
	cur_action = action;
}

void execute()
{
	switch(*command) {
		case C_STATE:
			sendState();
			break;
		case C_STOP:
			stop();
			ok();
			break;
		case C_FORWARD:
			mv_forward(1000);
			ok();
			break;
		case C_BACK:
			mv_back(1000);
			ok();
			break;
		case C_TLEFT:
			turn_left(1000);
			ok();
			break;
		case C_TRIGHT:
			turn_right(1000);
			ok();
			break;
		case C_RESCNT:
			lastLCount = lastRCount = motor1QeiCounts = motor2QeiCounts = 0;
			ok();
			break;
		case C_MCALIB:
			calibrate_motors();
			ok();
			break;
		case C_SETACC:
			stop_acc_x = adxl345_state.event.acceleration.x;
			stop_acc_y = adxl345_state.event.acceleration.y;
			stop_acc_z = adxl345_state.event.acceleration.z;
			ok();
			break;
		case C_SPIN:
			start_action(new Spinning);
			ok();
			break;
		case C_TURN2HEAD:
			start_action(new TurnToHeading(*(uint16_t*)dataBuf));
			ok();
			break;
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

	if(reqComplete) {
		reqComplete = false;
		execute();
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
	delay(5);
}
