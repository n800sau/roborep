#include <IRremote.h>

#include "../../include/common.h"

#define LEFT_ENCODER A1
#define RIGHT_ENCODER A0

#define LEFT_IR_PIN A5
#define RIGHT_IR_PIN A4

IRrecv left_ir(LEFT_IR_PIN);
IRrecv right_ir(RIGHT_IR_PIN);

#define LED_PIN 13
bool blinkState = false;

#define RIGHT_LED_PIN 12
#define LEFT_LED_PIN 11

unsigned long left_in_view=0, right_in_view=0;
const int timeout = 1000;
bool last_left = false, last_right = false;

void setup()
{
	Serial.begin(57600);

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);

	pinMode(RIGHT_LED_PIN, OUTPUT);
	pinMode(LEFT_LED_PIN, OUTPUT);

	pinMode(RIGHT_IR_PIN, INPUT);
	pinMode(LEFT_IR_PIN, INPUT);

	left_ir.enableIRIn(); // Start the receiver
	right_ir.enableIRIn(); // Start the receiver
}

void dump(decode_results *results) 
{
	int count = results->rawlen;
	if (results->decode_type == UNKNOWN) {
		Serial.print("Unknown encoding: ");
	} else {
		if (results->decode_type == NEC) {
			Serial.print("Decoded NEC: ");
		} else if (results->decode_type == SONY) {
			Serial.print("Decoded SONY: ");
		} else if (results->decode_type == RC5) {
			Serial.print("Decoded RC5: ");
		} else if (results->decode_type == RC6) {
			Serial.print("Decoded RC6: ");
		} else if (results->decode_type == PANASONIC) {
			Serial.print("Decoded PANASONIC - Address: ");
			Serial.print(results->panasonicAddress,HEX);
			Serial.print(" Value: ");
		} else if (results->decode_type == JVC) {
			Serial.print("Decoded JVC: ");
		}
		Serial.print(results->value, HEX);
		Serial.print(" (");
		Serial.print(results->bits, DEC);
		Serial.println(" bits)");
		Serial.print("Raw (");
		Serial.print(count, DEC);
		Serial.println("): ");

	}
//	for (int i = 0; i < count; i++) {
//		if ((i % 2) == 1) {
//		Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
//		} else {
//			Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
//		}
//		Serial.print(" ");
//	}
	Serial.println("");
}

bool ir_process(IRrecv &ir, const char *label)
{
	bool rs = false;
	decode_results results;
	if (ir.decode(&results)) {
//		dump(&results);
		if(results.decode_type == NEC) {
			Serial.print(label);
			Serial.print("\t");
			Serial.println(results.value, HEX);
			if(results.value == BEACON_CODE) {
				rs = true;
			}
				rs = true;
		}
		ir.resume(); // Receive the next value
	}
	return rs;
}

void loop() 
{
	unsigned long curr_millis = millis();
	if( ir_process(left_ir, "left") ) {
		left_in_view = curr_millis;
	}
	if( ir_process(right_ir, "right") ) {
		right_in_view = curr_millis;
	}
	unsigned long endtime = curr_millis - timeout;
	bool left = left_in_view > endtime;
	bool right = right_in_view > endtime;
	digitalWrite(LEFT_LED_PIN, left);
	digitalWrite(RIGHT_LED_PIN, right);
	if(left != last_left || right != last_right) {
		Serial.print(endtime);
		Serial.print(" ");
		Serial.print(left);
		Serial.print(" ");
		Serial.println(right);
	}
	last_left = left;
	last_right = right;

	digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

