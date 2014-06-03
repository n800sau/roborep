#include <IRremote.h>

#define RECV_PIN A5

IRrecv irrecv(RECV_PIN);

decode_results results;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup() {
	Serial.begin(57600);

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);

	irrecv.enableIRIn(); // Start the receiver

}

void dump(decode_results *results) 
{
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
//	  Serial.print("Unknown encoding: ");
  } else {
  if (results->decode_type == NEC) {
	Serial.print("Decoded NEC: ");
  } 
  else if (results->decode_type == SONY) {
	Serial.print("Decoded SONY: ");
  } 
  else if (results->decode_type == RC5) {
	Serial.print("Decoded RC5: ");
  } 
  else if (results->decode_type == RC6) {
	Serial.print("Decoded RC6: ");
  }
  else if (results->decode_type == PANASONIC) {	
	Serial.print("Decoded PANASONIC - Address: ");
	Serial.print(results->panasonicAddress,HEX);
	Serial.print(" Value: ");
  }
  else if (results->decode_type == JVC) {
	 Serial.print("Decoded JVC: ");
  }
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  for (int i = 0; i < count; i++) {
	if ((i % 2) == 1) {
	  Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
	} 
	else {
	  Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
	}
	Serial.print(" ");
  }
  Serial.println("");
  }
}

void loop() 
{
	if (irrecv.decode(&results)) {
		dump(&results);
		if(results.decode_type == NEC) {
			Serial.print("ir\t");
			Serial.print(results.value);
			Serial.println();
		}
		irrecv.resume(); // Receive the next value
	} else {
		digitalWrite(LED_PIN, HIGH);
		delay(10);
		digitalWrite(LED_PIN, LOW);
		delay(100);
	}
}

