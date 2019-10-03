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

void SerialProtocol::sendBuffer(byte cmd, byte *data, byte size)
{
	byte bbuf[3 + 4 + size + 1];
	bbuf[0] = MAGIC_BYTE;
	bbuf[1] = cmd;
	bbuf[2] = size + 4;
	(*(uint32_t*)(bbuf+3)) = millis();
	if(size>0) {
		memcpy(bbuf+7, data, size);
	}
	bbuf[3+4+size] = crc8(bbuf, 3+4+size);
	Serial.write(bbuf, 3 + 4 + size + 1);
}


void SerialProtocol::sendSimple(byte cmd)
{
	sendBuffer(cmd, NULL, 0);
}

void SerialProtocol::sendFloats(byte cmd, float vals[], int count)
{
	byte bbuf[sizeof(float) * count];
	for(int i=0; i<count; i++) {
		*((float*)(bbuf+sizeof(float)*i)) = vals[i];
	}
	sendBuffer(cmd, bbuf, count*sizeof(float));
}


void SerialProtocol::resetInput()
{
	// too much of bytes
	// cancel it
	inputPos = -1;
	reqComplete = false;
}

