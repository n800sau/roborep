float thermistor;
float terms[8][8];

const byte eol[3] = {'*', '*', '*'};
const float multiplier_tarr = 0.25;
const float multiplier_th = 0.0125;

union {
	uint8_t bytes[200];
	struct __attribute__((packed)) {
		int16_t thermistor;
		int16_t terms[8][8];
	} t;
} linebuf;

unsigned serial_readline()
{
	unsigned pos = 0;
	memset(&linebuf, 0, sizeof(linebuf));
	if(Serial.available()) {
		while(true) {
			if(Serial.available()) {
				byte b = Serial.read();
				linebuf.bytes[pos++] = b;
				if(pos >= sizeof(eol) && memcmp(linebuf.bytes+pos-sizeof(eol), eol, sizeof(eol)) == 0) {
					pos -= sizeof(eol);
					if(pos < 132) {
						pos = 0;
						continue;
					}
					break;
				}
				if(pos >= sizeof(linebuf.bytes)) {
					pos = 0;
					break;
				}
			}
		}
	}
	return pos;
}

bool refresh_terms()
{
	bool rs = false;
	int len = serial_readline();
	rs = len == 132;
	if(rs) {
		if((linebuf.bytes[1] & 0x8) == 0) {
			linebuf.bytes[1] &= 0x7;
			linebuf.t.thermistor = -linebuf.t.thermistor;
		}
		thermistor = linebuf.t.thermistor * multiplier_th;
		for(int r=0; r<8; r++) {
			for(int c=0; c<8; c++) {
				int i = 2 + (8 * r + c) * 2;
				if((linebuf.bytes[i+1] & 0x8) != 0) {
					linebuf.bytes[i+1] |= 0xf8;
				}
				terms[r][c] = linebuf.t.terms[r][c] * multiplier_tarr;
				test_touch();
//				Serial.print(int(terms[r][c]), DEC);
//				Serial.print(" ");
			}
//			Serial.println();
		}
//		Serial.println();
	}
	return rs;
}
