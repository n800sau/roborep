#ifndef __SERIALPROTOCOL_H

#define __SERIALPROTOCOL_H

#include "Arduino.h"

#define MAGIC_BYTE 0x85

class SerialProtocol {

	protected:
		boolean reqComplete; // false

	public:
		SerialProtocol();

// format
// 1b magic_byte(0x85)
// 1b command
// 1b data size without crc
// <size>b data
// 1b crc of 

		int inputPos;        // -1
		// data
		byte reqBuf[266];    // command,datasize,data
		byte *command;       // reqBuf;
		byte *datasize;      // reqBuf+1
		byte *dataBuf;       // reqBuf+2

		bool available();
		void serialEvent();
		void sendSimple(byte cmd);
		void sendFloats(byte cmd, float vals[], int count);
		void resetInput();

};

#endif //__SERIALPROTOCOL_H
