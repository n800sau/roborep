#define PIN_RESET 10

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
	delay(500);
}

void flushSerial3()
{
	while(Serial3.available() > 0) Serial3.read();
}

int waitForBytes(int count)
{
	int rs;
	while((rs = Serial3.available()) < count)
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
	if(cline.idx < cline.n) {
		rs = cline.vals[cline.idx];
	}
	cline.idx++;
	Serial.println(rs, HEX);
	return rs;
}

void loop()
{
	end_of_data = false;
	if(Serial.available()) {
		int cmd = Serial.read();
		switch(cmd)
		{
			case '1':
				slave_baud = 19200;
				break;
			case '2':
				slave_baud = 38400;
				break;
			case '3':
				slave_baud = 57600;
				break;
			case '4':
				slave_baud = 115200;
				break;
			case 'c':
				Serial.println("Starting...");
				Serial.println(slave_baud);
				Serial3.begin(slave_baud);
				resetSlave();
				flushSerial3();
				Serial.println("syncing...");
				Serial3.write(0x30);
				Serial3.write(0x20);
				delay(500);
				waitForBytes(2);
				int insync = Serial3.read();
				int ok = Serial3.read();
				if(in_sync(insync, ok)) {
					//receive major version
					Serial3.write(0x41);
					Serial3.write(0x81);
					Serial3.write(0x20);
					waitForBytes(3);
					insync = Serial3.read();
					int stk_major = Serial3.read(); // STK_SW_MAJOR
					ok = Serial3.read();
					if(in_sync(insync, ok)) {
						//receive minor version
						Serial3.write(0x41);
						Serial3.write(0x82);
						Serial3.write(0x20);
						waitForBytes(3);
						insync = Serial3.read();
						int stk_minor = Serial3.read(); // STK_SW_MINOR
						ok = Serial3.read();
						if(in_sync(insync, ok)) {
							Serial.print("bootloader version ");
							Serial.print(stk_major);
							Serial.print(".");
							Serial.println(stk_minor);
							Serial.println("Entering prog mode");
							Serial3.write(0x50);
							Serial3.write(0x20);
							waitForBytes(2);
							insync = Serial3.read();
							ok = Serial3.read();
							if(in_sync(insync, ok)) {
								Serial.println("Receiving signature");
								Serial3.write(0x75);
								Serial3.write(0x20);
								waitForBytes(5);
								insync = Serial3.read();
								int stk_signature[3];
								stk_signature[0] = Serial3.read();
								stk_signature[1] = Serial3.read();
								stk_signature[2] = Serial3.read();
								ok = Serial3.read();
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
									while(!end_of_data) {
										read_next_line();
										if(!end_of_data) {
											Serial.println("Writing next line");
											unsigned address = cline.addr;
											unsigned laddress = address & 0xFF;
											unsigned haddress = address >> 8 & 0xff;
											Serial3.write(0x55);
											Serial3.write(laddress);
											Serial3.write(haddress);
											Serial3.write(0x20);
											waitForBytes(2);
											insync = Serial3.read();
											ok = Serial3.read();
											if(in_sync(insync, ok)) {
												Serial3.write(0x64); // STK_PROGRAM_PAGE
												Serial3.write(0); // page size
												Serial3.write(cline.n); // page size
												Serial3.write(0x46); // flash memory, 'F'
												for(int i=0; i<cline.n; i++) {
													b = read_byte();
													Serial3.write(b);
												}
												Serial3.write(0x20); // SYNC_CRC_EOP
												waitForBytes(2);
												insync = Serial3.read();
												ok = Serial3.read();
												if(!in_sync(insync, ok)) {
													break;
												}
											}
										}
									}
									if(end_of_data) {
										// end of file
										Serial.println("Leaving prog mode");
										Serial3.write(0x51);
										Serial3.write(0x20);
										waitForBytes(2);
										insync = Serial3.read();
										ok = Serial3.read();
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
				break;
		}
	}
}
