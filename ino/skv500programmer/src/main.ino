#define PIN_RESET 8

HardwareSerial *pSerial = &Serial2;

long slave_baud = 115200;

void setup()
{
	Serial.begin(115200);
	Serial.println("Ready");
	pinMode(PIN_RESET, OUTPUT);
	digitalWrite(PIN_RESET, HIGH);
}

void resetSlave()
{
	digitalWrite(PIN_RESET, LOW);
	delay(500);
	digitalWrite(PIN_RESET, HIGH);
	delay(100);
}

void flushOutSerial()
{
	while(pSerial->available() > 0) pSerial->read();
}

int waitForBytes(int count)
{
	int rs;
	while((rs = pSerial->available()) < count)
	{
		delay(10);
	};
	Serial.print("Available: ");
	Serial.print(rs);
	Serial.println(" bytes");
	return rs;
}

bool in_sync(char fb, char lb)
{
	int rs = fb == 0x14 && lb == 0x10;
	if(!rs) {
		Serial.print("Received: 0x");
		Serial.print(fb, HEX);
		Serial.print(" and 0x");
		Serial.println("OUT of sync");
	}
	return rs;
}

#define PAGE_SIZE (8*16)
#define VALS_COUNT PAGE_SIZE

struct _CUR_LINE {
	unsigned addr;
	unsigned type;
	int n;
	unsigned vals[VALS_COUNT];
	int idx;
} cline;

bool end_of_data;

void read_next_line()
{
	String line;
	char snum[5];
	const char *p;
	int i, n;
	unsigned crc, val;
	// read next line;
	Serial.println("Give line");
	line = Serial.readStringUntil('\n');
	line.trim();
	Serial.print("Line:");
	Serial.println(line);
	if(line.length() > 0) {
		p = line.c_str();
		crc = 0;
		snum[0] = p[7];
		snum[1] = p[8];
		snum[2] = 0;
		cline.type = (int)strtol(snum, NULL, 16);
		crc += cline.type;
		if(cline.type == 0) { // record type - (0-data, 1-end)
			snum[0] = p[1];
			snum[1] = p[2];
			snum[2] = 0;
			n = (int)strtol(snum, NULL, 16);
			crc += n;
			if(n > VALS_COUNT) {
				Serial.print("Error. Hex line is too long ");
				Serial.print(n);
				Serial.print(" > ");
				Serial.println(VALS_COUNT);
			} else {
				snum[0] = p[3];
				snum[1] = p[4];
				snum[2] = p[5];
				snum[3] = p[6];
				snum[4] = 0;
				cline.addr = (int)strtol(snum, NULL, 16);
				crc += cline.addr & 0xff;
				crc += (cline.addr >> 8) & 0xff;
				Serial.print("lineaddr=0x");
				Serial.println(cline.addr, HEX);
				// skip 9 byte of : and address etc
				p += 9;
				for(i=0; i<n; i++) {
					snum[0] = p[0];
					snum[1] = p[1];
					snum[2] = 0;
					val = (int)strtol(snum, NULL, 16);
					crc += val;
					cline.vals[i] = val;
					p += 2;
				}
				// check crc
				snum[0] = p[0];
				snum[1] = p[1];
				snum[2] = 0;
				val = (unsigned)strtol(snum, NULL, 16);
				crc = ((crc ^ 0xff) + 1) & 0xff;
				if(crc != val) {
					Serial.print("CRC error: ");
				}
				Serial.print("crc=");
				Serial.print(crc, HEX);
				Serial.print(" vs val=");
				Serial.println(val, HEX);
				Serial.print("count=");
				Serial.println(n);
				cline.n = n;
				cline.idx = 0;
			}
		} else {
			Serial.println("End of data");
			end_of_data = true;
		}
	} else {
		Serial.println("No more data");
		end_of_data = true;
	}
}

// return -1 if error or end of vals
int read_byte()
{
	int rs = -1;
	if(cline.idx >= cline.n) {
		if(!end_of_data) {
			read_next_line();
		}
	}
	if(cline.idx < cline.n && !end_of_data) {
		rs = cline.vals[cline.idx++];
	}
	Serial.println(rs, HEX);
	return rs;
}

void stk500program()
{
	resetSlave();
	flushOutSerial();
	Serial.println("syncing...");
	pSerial->write(0x30);
	pSerial->write(0x20);
	delay(500);
	waitForBytes(2);
	int insync = pSerial->read();
	int ok = pSerial->read();
	if(in_sync(insync, ok)) {
		//receive major version
		pSerial->write(0x41);
		pSerial->write(0x81);
		pSerial->write(0x20);
		waitForBytes(3);
		insync = pSerial->read();
		int stk_major = pSerial->read(); // STK_SW_MAJOR
		ok = pSerial->read();
		if(in_sync(insync, ok)) {
			//receive minor version
			pSerial->write(0x41);
			pSerial->write(0x82);
			pSerial->write(0x20);
			waitForBytes(3);
			insync = pSerial->read();
			int stk_minor = pSerial->read(); // STK_SW_MINOR
			ok = pSerial->read();
			if(in_sync(insync, ok)) {
				Serial.print("bootloader version ");
				Serial.print(stk_major);
				Serial.print(".");
				Serial.println(stk_minor);
				Serial.println("Entering prog mode");
				pSerial->write(0x50);
				pSerial->write(0x20);
				waitForBytes(2);
				insync = pSerial->read();
				ok = pSerial->read();
				if(in_sync(insync, ok)) {
					Serial.println("Receiving signature");
					pSerial->write(0x75);
					pSerial->write(0x20);
					waitForBytes(5);
					insync = pSerial->read();
					int stk_signature[3];
					stk_signature[0] = pSerial->read();
					stk_signature[1] = pSerial->read();
					stk_signature[2] = pSerial->read();
					ok = pSerial->read();
					if(in_sync(insync, ok)) {
						Serial.print("Signature ");
						Serial.print(stk_signature[0]);
						Serial.print("-");
						Serial.print(stk_signature[1]);
						Serial.print("-");
						Serial.println(stk_signature[2]);
						Serial.println("Proceed");
						cline.n=0;
						cline.idx=0;
						int b;
						read_next_line();
						unsigned address = cline.addr;
						while(!end_of_data) {
								Serial.print("Address: 0x");
								Serial.println(address, HEX);
								unsigned laddress = address & 0xff;
								unsigned haddress = address >> 8 & 0xff;
								pSerial->write(0x55);
								pSerial->write(laddress);
								pSerial->write(haddress);
								pSerial->write(0x20);
								address += PAGE_SIZE >> 1;
								waitForBytes(2);
								insync = pSerial->read();
								ok = pSerial->read();
								if(in_sync(insync, ok)) {
									pSerial->write(0x64); // STK_PROGRAM_PAGE
									pSerial->write(0); // page size
									pSerial->write(PAGE_SIZE); // page size
									pSerial->write(0x46); // flash memory, 'F'
									for(int i=0; i<PAGE_SIZE; i++) {
										b = read_byte();
										pSerial->write(b);
									}
									pSerial->write(0x20); // SYNC_CRC_EOP
									waitForBytes(2);
									insync = pSerial->read();
									ok = pSerial->read();
									if(!in_sync(insync, ok)) {
										break;
									}
								}
						}
						if(end_of_data) {
							// end of file
							Serial.println("Leaving prog mode");
							pSerial->write(0x51);
							pSerial->write(0x20);
							waitForBytes(2);
							insync = pSerial->read();
							ok = pSerial->read();
							if(in_sync(insync, ok)) {
								delay(1000);
								resetSlave();
								Serial.println("Finished");
							}
						}
					}
				}
			}
		}
	}
	Serial.println("Stopped");
}

bool pass_through_mode = false;

void resetOutSerial()
{
	Serial.print("Slave is Serial");
	if(pSerial == &Serial1) {
		Serial.print("1");
	} else if(pSerial == &Serial2) {
		Serial.print("2");
	} else if(pSerial == &Serial3) {
		Serial.print("3");
	}
	Serial.print(" with baud ");
	Serial.println(slave_baud);
	pSerial->begin(slave_baud);
}

void loop()
{
	if(pass_through_mode) {
		int n = pSerial->available();
		for(int i=0; i< n; i++) {
			Serial.write(pSerial->read());
		}
		n = Serial.available();
		for(int i=0; i< n; i++) {
			pSerial->write(Serial.read());
		}
	} else {
		end_of_data = false;
		if(Serial.available()) {
			int cmd = Serial.read();
			switch(cmd)
			{
				case '!':
					pSerial = &Serial1;
					resetOutSerial();
					break;
				case '@':
					pSerial = &Serial2;
					resetOutSerial();
					break;
				case '#':
					pSerial = &Serial3;
					resetOutSerial();
					break;
				case '1':
					slave_baud = 19200;
					resetOutSerial();
					break;
				case '2':
					slave_baud = 38400;
					resetOutSerial();
					break;
				case '3':
					slave_baud = 57600;
					resetOutSerial();
					break;
				case '4':
					slave_baud = 115200;
					resetOutSerial();
					break;
				case 'c':
					resetOutSerial();
					stk500program();
					break;
				case 'p':
					resetOutSerial();
					resetSlave();
					flushOutSerial();
					while(pSerial->available() > 0 && pSerial->peek() == 0) {
						pSerial->read();
					}
					pass_through_mode = true;
					Serial.println("Pass through mode is set");
					break;
			}
		}
	}
}
