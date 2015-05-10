#include "SerialProtocol.h"
#include <crc.h>

SerialProtocol::SerialProtocol():
	inputPos(-1),
	command(reqBuf),
	datasize(reqBuf+1),
	dataBuf(reqBuf+2),
	reqComplete(false)
{
}

bool SerialProtocol::available()
{
	return reqComplete;
}

void SerialProtocol::serialEvent()
{
	while (Serial.available() && !reqComplete) {
		byte inChar = Serial.read();
		if( inputPos < 0 && inChar == MAGIC_BYTE ) {
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

void SerialProtocol::sendSimple(byte cmd)
{
	byte bbuf[4];
	bbuf[0] = MAGIC_BYTE;
	bbuf[1] = cmd;
	bbuf[2] = 0;
	bbuf[3] = crc8(bbuf, 3);
	Serial.write(bbuf, 4);
}

void SerialProtocol::sendFloats(byte cmd, float vals[], int count)
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


void SerialProtocol::resetInput()
{
	// too much of bytes
	// cancel it
	inputPos = -1;
	reqComplete = false;
}

