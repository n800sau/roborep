#include <crc.h>

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte crc8(const byte *data, int len) {
	byte crc = 0;
	while(len--) {
		byte extract = *data++;
		for(int i = 0; i < 8; i++) {
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= 0x8C;
			}
			extract >>= 1;
		}
	}
	return crc;
}
