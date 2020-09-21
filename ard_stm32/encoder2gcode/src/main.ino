#include <USBComposite.h>
#include <RotaryEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define USBSerialIndex 0

// USB - PA11,PA12
USBMultiSerial<1> ms;

#define CNCSerial Serial3
#define DBGSerial Serial
#define USBSerial ms.ports[USBSerialIndex]
//#define USBSerial DBGSerial

int16_t x_pos = 0, y_pos = 0, z_pos = 0;
float x_fpos = 0, y_fpos = 0, z_fpos = 0;
float x_frate = 1, y_frate = 1, z_frate = 1;

struct Status_T {
	String state;
	float mXpos, mYpos, mZpos;
	float wXpos, wYpos, wZpos;
	String limit_state;
} status = {
	.state = "",
	.mXpos = 0, .mYpos = 0, .mZpos = 0, .wXpos = 0, .wYpos = 0, .wZpos = 0,
	.limit_state = ""
};

enum eMODE {EM_POS, EM_RATE};

// p1, p2, button
const int xp1=PB14, xp2=PB13, xb=PB12;
RotaryEncoder encoderX(xp1, xp2, xb);
eMODE xMode = EM_POS;

const int yp1=PA2, yp2=PA3, yb=PA4;
RotaryEncoder encoderY(yp1, yp2, yb);
eMODE yMode = EM_POS;

const int zp1=PB9, zp2=PB8, zb=PB7;
RotaryEncoder encoderZ(zp1, zp2, zb);
eMODE zMode = EM_POS;

#define STOP_BUTTON_PIN PB15
volatile bool stop_marker = false;

#define PAUSE_BUTTON_PIN PA8
volatile bool pause_marker = false;
bool paused = false;

#define UNLOCK_BUTTON_PIN PA6
volatile bool unlock_marker = false;

#define TFT_CS PB15
#define TFT_DC PB1
#define TFT_RST PB0
#define TFT_MOSI PA7
#define TFT_SCLK PA5

// colored 160x80 0.96"
//Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
// SPI version much faster
SPIClass SPI_1(1);
Adafruit_ST7735 display = Adafruit_ST7735(&SPI_1, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas1 dbuf(160, 80);

#define LED_PIN LED_BUILTIN

void XencoderISR()
{
	encoderX.readAB();
}

void XencoderButtonISR()
{
	encoderX.readPushButton();
}

void YencoderISR()
{
	encoderY.readAB();
}

void YencoderButtonISR()
{
	encoderY.readPushButton();
}

void ZencoderISR()
{
	encoderZ.readAB();
}

void ZencoderButtonISR()
{
	encoderZ.readPushButton();
}

void StopISR()
{
	stop_marker = true;
}

void PauseISR()
{
	pause_marker = true;
}

void UnlockISR()
{
	unlock_marker = true;
}

void update_display()
{
	dbuf.fillScreen(0);
	int y = 5, x[] = {10, 80};
	const int y_step = 20;
	dbuf.setCursor(x[0], y);
	dbuf.print(status.state);
	y += y_step;
	dbuf.setCursor(x[0], y);
	dbuf.print(F("X: "));
	dbuf.print(status.mXpos);
//	dbuf.print(x_fpos);
	dbuf.print(F(","));
	dbuf.setCursor(x[1], y);
	dbuf.print(xMode ? F("*") : F(" "));
	dbuf.print(x_frate);
	y += y_step;
	dbuf.setCursor(x[0], y);
	dbuf.print(F("Y: "));
	dbuf.print(status.mYpos);
//	dbuf.print(y_fpos);
	dbuf.print(F(","));
	dbuf.setCursor(x[1], y);
	dbuf.print(yMode ? F("*") : F(" "));
	dbuf.print(y_frate);
	y += y_step;
	dbuf.setCursor(x[0], y);
	dbuf.print(F("Z :"));
	dbuf.print(status.mZpos);
//	dbuf.print(z_fpos);
	dbuf.print(F(","));
	dbuf.setCursor(x[1], y);
	dbuf.print(zMode ? F("*") : F(" "));
	dbuf.print(z_frate);
	display.drawBitmap(0, 0, dbuf.getBuffer(), dbuf.width(), dbuf.height(), ST7735_YELLOW, ST7735_BLACK);
}

void setup()
{
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
	pinMode(PAUSE_BUTTON_PIN, INPUT_PULLUP);
	pinMode(UNLOCK_BUTTON_PIN, INPUT_PULLUP);

	SPI_1.begin(); //Initialize the SPI_2 port.
//	SPI_1.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
//	SPI_1.setDataMode(SPI_MODE0); //Set the	SPI_2 data mode 0
//	SPI_1.setClockDivider(SPI_CLOCK_DIV16);	// Use a different speed to SPI 1

	DBGSerial.begin(115200);
	DBGSerial.println("Hello from encoder2gcode");
	DBGSerial.flush();

	display.initR(INITR_MINI160x80); // initialize a ST7735S chip
	display.setTextWrap(false); // Allow text to run off right edge
	display.fillScreen(ST7735_BLACK);
	display.setRotation(3);
	display.setTextSize(1);
	display.invertDisplay(true);
	display.setTextColor(ST77XX_WHITE);
	update_display();

	ms.begin();
	//while(!USBComposite);


	encoderX.begin();
	encoderY.begin();
	encoderZ.begin();

	attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), StopISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(PAUSE_BUTTON_PIN), PauseISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(UNLOCK_BUTTON_PIN), UnlockISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(xp1), XencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(xp2), XencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(xb), XencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(yp1), YencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(yp2), YencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(yb), YencoderButtonISR, FALLING);

	attachInterrupt(digitalPinToInterrupt(zp1), ZencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(zp2), ZencoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(zb), ZencoderButtonISR, FALLING);

	CNCSerial.begin(115200);
}

bool encoder_update(RotaryEncoder &enc, int16_t &oldpos, eMODE mode, float &fpos, float &frate)
{
	int16_t pos = enc.getPosition();

	float pdiff = pos - oldpos;
	if(mode == EM_RATE) {
		frate += frate * pdiff/10.;
		if(frate < 0.1) {
			frate = 0.1;
		}
	} else {
		fpos += pdiff * frate;
	}
	oldpos = pos;
	return bool(pdiff);
}

unsigned long m = millis();
uint8_t cnc_reply_buf[256];
char status_process_buf[sizeof(cnc_reply_buf)];
unsigned cnc_counter = 0;
String jog_command;
#define IDLE_MARKER "<Idle"
#define JOG_MARKER "<Jog"

void loop()
{

	bool changed = false;

	if(encoderX.getPushButton()) {
		changed = true;
		Serial.println("X pressed");
		xMode = xMode == EM_POS ? EM_RATE : EM_POS;
	}

	if(encoder_update(encoderX, x_pos, xMode, x_fpos, x_frate))
	{
		changed = true;
		DBGSerial.print("Xpos:");
		DBGSerial.println(x_fpos);
		DBGSerial.print("Xrate:");
		DBGSerial.println(x_frate);
	}

	if(encoderY.getPushButton()) {
		changed = true;
		yMode = yMode == EM_POS ? EM_RATE : EM_POS;
		Serial.println("Y pressed");
	}

	if(encoder_update(encoderY, y_pos, yMode, y_fpos, y_frate))
	{
		changed = true;
		DBGSerial.print("Ypos:");
		DBGSerial.println(y_fpos);
		DBGSerial.print("Yrate:");
		DBGSerial.println(y_frate);
	}

	if(encoderZ.getPushButton() == true) {
		changed = true;
		zMode = zMode == EM_POS ? EM_RATE : EM_POS;
		Serial.println("Z pressed");
	}

	if(encoder_update(encoderZ, z_pos, zMode, z_fpos, z_frate))
	{
		changed = true;
		DBGSerial.print("Zpos:");
		DBGSerial.println(z_fpos);
		DBGSerial.print("Zrate:");
		DBGSerial.println(z_frate);
	}

	if(changed) {
		jog_command = String("$J=G91 X") + x_fpos + " Y" + y_fpos + " Z" + z_fpos + " F100";
		CNCSerial.print("?");
		CNCSerial.flush();
	}

	if(stop_marker) {
		stop_marker = false;
//		CNCSerial.println("\nM112");
		// sort reset
		CNCSerial.print("\030");
		CNCSerial.flush();
	}

	if(unlock_marker) {
		unlock_marker = false;
		CNCSerial.println("$X");
		CNCSerial.flush();
	}

	if(pause_marker) {
		pause_marker = false;
		if(paused) {
			paused = false;
		} else {
			paused = true;
		}
		CNCSerial.print(paused ? "!" : "~");
		CNCSerial.flush();
	}

	while(USBSerial.available() || CNCSerial.available()) {
		digitalWrite(LED_PIN, LOW);
		if(USBSerial.available()) {
			CNCSerial.write(USBSerial.read());
		}
		if(CNCSerial.available()) {
			uint8_t c = CNCSerial.read();
			USBSerial.write(c);
			cnc_reply_buf[cnc_counter++] = c;
			if(cnc_counter >= sizeof(cnc_reply_buf) || c == (uint8_t)0xa) {
				grblStatusEvaluation(cnc_reply_buf, cnc_counter);
				if(jog_command != "") {
					if(memcmp(cnc_reply_buf, IDLE_MARKER, min(cnc_counter, strlen(IDLE_MARKER))) == 0 || memcmp(cnc_reply_buf, JOG_MARKER, min(cnc_counter, strlen(JOG_MARKER))) == 0) {
						DBGSerial.print("Sending:");
						DBGSerial.println(jog_command);
						CNCSerial.println(jog_command);
						jog_command = "";
						// here pos must be reset
						encoderX.setPosition(0);
						encoderY.setPosition(0);
						encoderZ.setPosition(0);
						changed = true;
					}
				}
				cnc_counter = 0;
			}
		}
	}
	digitalWrite(LED_PIN, HIGH);
	if(changed) {
		update_display();
	}

//	if(m+1000 < millis()) {
//		m = millis();
//		digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//	}
}

/*------------------------------------------------------------------------------
	The status line can take these various states:
	$10=1
	<Idle|MPos:0.000,0.000,0.000|FS:0,0|WCO:0.000,0.000,0.000>
	<Idle|MPos:0.000,0.000,0.000|FS:0,0|Ov:100,100,100>
	<Idle|MPos:0.000,0.000,0.000|FS:0,0>
	$10=2
	<Idle|WPos:0.000,0.000,0.000|Bf:15,128|FS:0,0|WCO:0.000,0.000,0.000>
	<Idle|WPos:0.000,0.000,0.000|Bf:15,128|FS:0,0|Ov:100,100,100>
	<Idle|WPos:0.000,0.000,0.000|Bf:15,128|FS:0,0>
--------------------------------------------------------------------------------
*/
void grblStatusEvaluation(uint8_t *buf, int count)
{
	uint8_t *start_pos = (uint8_t*)memchr(buf, '<', count);
	if(start_pos)												// Status GRBL
	{
		uint8_t *end_pos = (uint8_t*)memchr(buf, '>', count);
		if(end_pos) {
			memcpy(status_process_buf, start_pos+1, end_pos - start_pos - 1);
			status_process_buf[end_pos - start_pos - 1] = 0;
			char *p1 = status_process_buf;
			char *p2 = strchr(p1, '|');
			if(p2) {
				*p2 = 0;
				status.state = p1;
				while(p2) {
					p1 = p2 + 1;
					p2 = strchr(p1, '|');
					if(p2) {
						*p2 = 0;
					}
					if(strncmp(p1, "MPos:", 5) == 0) {
						// machine position
						p1 += 5;
						char *p3 = strchr(p1, ',');
						if(p3) {
							*p3 = 0;
							status.mXpos = atof(p1);
							p1 = p3 + 1;
							p3 = strchr(p1, ',');
							if(p3) {
								*p3 = 0;
								status.mYpos = atof(p1);
								p1 = p3 + 1;
								status.mZpos = atof(p1);
							}
						}
					} else if(strncmp(p1, "WPos:", 5) == 0) {
						// work position
						p1 += 5;
						char *p3 = strchr(p1, ',');
						if(p3) {
							*p3 = 0;
							status.wXpos = atof(p1);
							p1 = p3 + 1;
							p3 = strchr(p1, ',');
							if(p3) {
								*p3 = 0;
								status.wYpos = atof(p1);
								p1 = p3 + 1;
								status.wZpos = atof(p1);
							}
						}
					} else if(strncmp(p1, "Lim:", 4) == 0) {
						// work position
						p1 += 4;
						status.limit_state = p1;
					}
				}
			}
		}
	}
}
