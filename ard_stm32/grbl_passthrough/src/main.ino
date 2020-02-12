#include <USBComposite.h>

#define SERIAL_LINE_SIZE 64
#define CMD_START_CHAR 'N'
#define dstSerial Serial1

USBCompositeSerial uSerial;
int inByte;

void setup()
{
	Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
	uSerial.begin(115200);
	dstSerial.begin(115200);
}

char serial_buf[SERIAL_LINE_SIZE];
char char_count = 0;
bool cmd_mode = false;

void uSendError(const char *err)
{
	uSerial.print("Err:");
	uSerial.println(err);
}

void add2cmd_buf(char b)
{
	if(char_count<SERIAL_LINE_SIZE) {
		serial_buf[char_count] = b;
	} else {
		uSendError("Buffer error");
	}
}

void loop()
{
	while(dstSerial.available() || uSerial.available()) {
		if(uSerial.available()) {
			inByte = uSerial.read();
			bool eol = (inByte == '\n') || (inByte == '\r');
			if(cmd_mode) {
				if(!eol) {
					add2cmd_buf(inByte);
				}
			} else {
				if(char_count == 0 && inByte == CMD_START_CHAR) {
					cmd_mode = true;
					add2cmd_buf(inByte);
				} else {
					dstSerial.write(inByte);
				}
			}
			if(eol) {
				if(cmd_mode) {
					// do smth
					cmd_mode = false;
				}
				char_count = 0;
			} else {
				char_count++;
			}
		}
		while(dstSerial.available()) {
			inByte = dstSerial.read();
			uSerial.write(inByte);
		}
	}
}
