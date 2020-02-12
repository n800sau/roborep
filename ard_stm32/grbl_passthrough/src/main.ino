#include <USBComposite.h>
#include <BasicStepperDriver.h>
#include <string.h>

#define SERIAL_LINE_SIZE 64
#define CMD_START_CHAR 'N'

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

#define DIR_PIN PB5
#define STEP_PIN PB6
#define ENBL_PIN PB7

// Since microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

#define STEPS_PER_MM 100

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, ENBL_PIN);

#define dstSerial Serial1
#define pcSerial uSerial

USBCompositeSerial uSerial;
int inByte;

void setup()
{
	Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
	pcSerial.begin(115200);
	dstSerial.begin(115200);
	stepper.begin(120, MICROSTEPS);
//	stepper.setSpeedProfile(LINEAR_SPEED, 1000, 1000);
}

char serial_buf[SERIAL_LINE_SIZE];
int char_count = 0;
bool cmd_mode = false;

#define CMD_MOVE 1
#define CMD_RELAX 2

void uSendError(const char *err)
{
	pcSerial.print(String(CMD_START_CHAR));
	pcSerial.print("error:");
	pcSerial.println(err);
}

void add2cmd_buf(char b)
{
	if(char_count<SERIAL_LINE_SIZE) {
		serial_buf[char_count] = b;
	} else {
		uSendError("Buffer error");
	}
}

void processCmd()
{
	serial_buf[char_count] = 0;
	int cmd = atoi(serial_buf+1);
	char *pp = strchr(serial_buf, ' ');
	float off = pp ? atof(pp+1) : 0;
	switch(cmd) {
		case CMD_MOVE:
			// energize coils - the motor will hold position
			stepper.enable();
			stepper.move(off*STEPS_PER_MM*MICROSTEPS);
			break;
		case CMD_RELAX:
			// pause and allow the motor to be moved by hand
			stepper.disable();
			break;
	}
}

void loop()
{
	while(dstSerial.available() || pcSerial.available()) {
		if(pcSerial.available()) {
			inByte = pcSerial.read();
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
					processCmd();
					cmd_mode = false;
				}
				char_count = 0;
			} else {
				char_count++;
			}
		}
		while(dstSerial.available()) {
			inByte = dstSerial.read();
			pcSerial.write(inByte);
		}
	}
}
