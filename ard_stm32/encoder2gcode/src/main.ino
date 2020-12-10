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
uint8_t menu_x = 0, menu_z = 0;

const uint8_t memory_cell_count = 4;
const uint8_t command_cell_count = 1;
const uint8_t menu_count_z = 2;
const int menu_count_x[menu_count_z] = {memory_cell_count, command_cell_count};

struct t_pos {
	float x;
	float y;
	float z;
	bool enabled;
	t_pos():x(0),y(0),z(0),enabled(false) {}
} m_pos[memory_cell_count];

// buffer for cnc reply
uint8_t cnc_reply_buf[256];
// current pointer in cnc_reply_buf
unsigned cnc_counter = 0;
// buffer for cnc status line
char status_process_buf[sizeof(cnc_reply_buf)];
String jog_command;
#define IDLE_STATE "Idle"
#define JOG_STATE "Jog"

struct Status_T {
	String state;
	float mXpos, mYpos, mZpos;
	float wXpos, wYpos, wZpos;
	String limit_state;
} status = {
	.state = IDLE_STATE,
	.mXpos = 0, .mYpos = 0, .mZpos = 0, .wXpos = 0, .wYpos = 0, .wZpos = 0,
	.limit_state = "",
};

enum eMODE {EM_POS, EM_RATE};

// p1, p2, button
const int xp1=PB9, xp2=PB8, xb=PB7;
RotaryEncoder encoderX(xp1, xp2, xb);
eMODE xMode = EM_POS;

const int yp1=PA2, yp2=PA3, yb=PA4;
RotaryEncoder encoderY(yp1, yp2, yb);
eMODE yMode = EM_POS;

const int zp1=PB14, zp2=PB13, zb=PB12;
RotaryEncoder encoderZ(zp1, zp2, zb);
eMODE zMode = EM_POS;

// turn on/off own gcode and external gcode mode
#define MODE_BUTTON_PIN PA15
volatile bool bypass_mode = false;

#define MENU_BUTTON_PIN PB5
volatile bool menu_mode = false;

volatile bool debounce_set = false;

enum BTN_TYPES:uint8_t {BTN_X, BTN_Y, BTN_Z};


// brown
#define TFT_CS PB15
// orange
#define TFT_DC PB1
// yellow
#define TFT_RST PB0
// blue
#define TFT_MOSI PA7
// green
#define TFT_SCLK PA5
// blk - black
// gnd - white
// 3.3v - red

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

void ModeISR()
{
	if(!debounce_set) {
		bypass_mode = !bypass_mode;
		debounce_set = true;
		digitalWrite(LED_PIN, LOW);
	}
}

void MenuISR()
{
	if(!debounce_set) {
		menu_mode = !menu_mode;
		debounce_set = true;
	}
}

void update_display()
{
	dbuf.fillScreen(0);
	int y_step = 14; // 80/14 = 5
	if(menu_mode) {
		int y = 5, x[memory_cell_count+1] = {10, 60, 85, 110, 135};
		dbuf.setCursor(x[0], y);
		dbuf.print("Menu");
		y += y_step;
		if(menu_z == 0) {
			dbuf.setCursor(0, y);
			dbuf.print(">");
		}
		dbuf.setCursor(x[0], y);
		dbuf.print("Memory:");
		for(int i=0; i<memory_cell_count; i++) {
			dbuf.setCursor(x[1 + i], y);
			dbuf.print("M");
			dbuf.print(1+i);
			dbuf.print(m_pos[i].enabled ? "*" : " ");
			if(menu_x == i) {
				dbuf.setCursor(x[1 + i], y + y_step/2);
				dbuf.print("^");
			}
		}
		y += y_step;
		if(menu_z == 1) {
			dbuf.setCursor(0, y);
			dbuf.print(">");
		}
		dbuf.setCursor(x[0], y);
		dbuf.print("Action:");
		int xa[command_cell_count+1] = {10, 80};
		for(int i=0; i<command_cell_count; i++) {
			dbuf.setCursor(xa[1 + i], y);
			dbuf.print("Unlock");
		}
	} else {
		int y = 5, x[] = {10, 80};
		dbuf.setCursor(x[0], y);
		dbuf.print(status.state + (bypass_mode ? " (bypass)" : " (control)"));
		y += y_step;
		dbuf.setCursor(x[0], y);
		dbuf.print(F("limits: "));
		dbuf.print(status.limit_state);
		y += y_step;
		dbuf.setCursor(x[0], y);
		dbuf.print(F("X: "));
		dbuf.print(status.mXpos);
		dbuf.print(F(","));
		dbuf.setCursor(x[1], y);
		dbuf.print(xMode ? F("*") : F(" "));
		dbuf.print(x_frate);
		y += y_step;
		dbuf.setCursor(x[0], y);
		dbuf.print(F("Y: "));
		dbuf.print(status.mYpos);
		dbuf.print(F(","));
		dbuf.setCursor(x[1], y);
		dbuf.print(yMode ? F("*") : F(" "));
		dbuf.print(y_frate);
		y += y_step;
		dbuf.setCursor(x[0], y);
		dbuf.print(F("Z :"));
		dbuf.print(status.mZpos);
		dbuf.print(F(","));
		dbuf.setCursor(x[1], y);
		dbuf.print(zMode ? F("*") : F(" "));
		dbuf.print(z_frate);
	}
	display.drawBitmap(0, 0, dbuf.getBuffer(), dbuf.width(), dbuf.height(), ST7735_YELLOW, ST7735_BLACK);
}

void setup()
{
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
	pinMode(MENU_BUTTON_PIN, INPUT_PULLUP);

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

	attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), ModeISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(MENU_BUTTON_PIN), MenuISR, FALLING);

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
	int pdiff = pos - oldpos;
	if(pdiff) {
		if(mode == EM_RATE) {
			frate += frate * pdiff/10.;
			if(frate < 0.1) {
				frate = 0.1;
			}
		} else {
			fpos += pdiff * frate / 2;
		}
		oldpos = pos;
	}
	return bool(pdiff);
}

bool encoder_menu_update(RotaryEncoder &enc, int16_t &oldpos, uint8_t &menu_pos, uint8_t max_pos)
{
	int16_t pos = enc.getPosition();
Serial.print("pos:");
Serial.println(pos);
	int pdiff = (pos - oldpos) / 2;
	if(pdiff) {
		pdiff = pdiff > 0 ? 1 : -1;
Serial.print("pdiff:");
Serial.println(pdiff);
		menu_pos += pdiff;
		if(menu_pos < 0) {
			menu_pos = 0;
		} else if(menu_pos > max_pos) {
			menu_pos = max_pos;
		}
		oldpos = pos;
	}
	return bool(pdiff);
}

void menu_item_clicked(BTN_TYPES btn)
{
	switch(menu_z) {
		case 0: // memory
			switch(btn) {
				case BTN_X:
					// remember position
					m_pos[menu_x].x = status.mXpos;
					m_pos[menu_x].y = status.mYpos;
					m_pos[menu_x].z = status.mZpos;
					m_pos[menu_x].enabled = true;
					break;
				case BTN_Y:
					m_pos[menu_x].enabled = false;
					break;
				case BTN_Z:
					if(m_pos[menu_x].enabled) {
						CNCSerial.println(String("$J=X") + m_pos[menu_x].x + " Y" + m_pos[menu_x].y + " Z" + m_pos[menu_x].z + " F5000");
					}
					break;
			}
			break;
		case 1: // functions
			break;
	}
}

bool process_byte_from_cnc(char c)
{
	bool rs = false;
	cnc_reply_buf[cnc_counter++] = c;
	if(cnc_counter >= sizeof(cnc_reply_buf) || c == (uint8_t)0xa) {
		rs = grblStatusEvaluation(cnc_reply_buf, cnc_counter);
		cnc_counter = 0;
	}
	return rs;
}

bool old_bypass_mode = false;

void loop()
{

	bool changed = false;

	if(bypass_mode != old_bypass_mode) {
		Serial.print("bypass mode now:");
		Serial.println(bypass_mode);
		old_bypass_mode = bypass_mode;
	}

	if(!bypass_mode) {

		if(menu_mode) {

			if(encoderX.getPushButton()) {
				changed = true;
				Serial.println("X pressed");
				menu_item_clicked(BTN_X);
			}

			if(encoderY.getPushButton()) {
				changed = true;
				Serial.println("Y pressed");
				menu_item_clicked(BTN_Y);
			}

			if(encoderZ.getPushButton() == true) {
				changed = true;
				Serial.println("Z pressed");
				menu_item_clicked(BTN_Z);
			}

			if(encoder_menu_update(encoderX, x_pos, menu_x, menu_count_x[menu_z]-1))
			{
				changed = true;
				DBGSerial.print("Xpos:");
				DBGSerial.println(menu_x);
			}

			if(encoder_menu_update(encoderZ, z_pos, menu_z, menu_count_z-1))
			{
				changed = true;
				DBGSerial.print("Zpos:");
				DBGSerial.println(menu_z);
			}
			if(changed) {
				if(menu_x >= menu_count_x[menu_z]) {
					menu_x = menu_count_x[menu_z] - 1;
				}
					encoderX.setPosition(0);
					encoderY.setPosition(0);
					encoderZ.setPosition(0);
			}

		} else {

			if(encoderX.getPushButton()) {
				changed = true;
				xMode = xMode == EM_POS ? EM_RATE : EM_POS;
				Serial.println("X pressed");
			}

			if(encoderY.getPushButton()) {
				changed = true;
				yMode = yMode == EM_POS ? EM_RATE : EM_POS;
				Serial.println("Y pressed");
			}

			if(encoderZ.getPushButton() == true) {
				changed = true;
				zMode = zMode == EM_POS ? EM_RATE : EM_POS;
				Serial.println("Z pressed");
			}

			if(encoder_update(encoderX, x_pos, xMode, x_fpos, x_frate))
			{
				changed = true;
				DBGSerial.print("Xpos:");
				DBGSerial.println(x_fpos);
				DBGSerial.print("Xrate:");
				DBGSerial.println(x_frate);
			}

			if(encoder_update(encoderY, y_pos, yMode, y_fpos, y_frate))
			{
				changed = true;
				DBGSerial.print("Ypos:");
				DBGSerial.println(y_fpos);
				DBGSerial.print("Yrate:");
				DBGSerial.println(y_frate);
			}

			if(encoder_update(encoderZ, z_pos, zMode, z_fpos, z_frate))
			{
				changed = true;
				DBGSerial.print("Zpos:");
				DBGSerial.println(z_fpos);
				DBGSerial.print("Zrate:");
				DBGSerial.println(z_frate);
			}
		}

		CNCSerial.print("?");
		CNCSerial.flush();
	}

//	if(menu_mode) {
//		menu_mode = false;
//		DBGSerial.println("$X");
//		CNCSerial.println("$X");
//		CNCSerial.flush();
//	}

	if(bypass_mode) {
		while(USBSerial.available() || CNCSerial.available()) {
			digitalWrite(LED_PIN, LOW);
			if(USBSerial.available()) {
				CNCSerial.write(USBSerial.read());
			}
			if(CNCSerial.available()) {
				uint8_t c = CNCSerial.read();
				USBSerial.write(c);
				changed |= process_byte_from_cnc(c);
			}
		}
	} else {
		while(CNCSerial.available()) {
			digitalWrite(LED_PIN, LOW);
			uint8_t c = CNCSerial.read();
			changed |= process_byte_from_cnc(c);
			if(x_fpos || y_fpos || z_fpos) {
				if(!menu_mode) {
					jog_command = String("$J=G91 X") + x_fpos + " Y" + y_fpos + " Z" + z_fpos + " F5000";
					x_fpos = y_fpos = z_fpos = 0;
					if(status.state == IDLE_STATE || status.state == JOG_STATE) {
						DBGSerial.print("Sending:");
						DBGSerial.println(jog_command);
						CNCSerial.println(jog_command);
						jog_command = "";
						changed = true;
					}
//					encoderX.setPosition(0);
//					encoderY.setPosition(0);
//					encoderZ.setPosition(0);
				}
			}
		}
	}
	digitalWrite(LED_PIN, HIGH);
	if(changed || debounce_set) {
		update_display();
		if(debounce_set) {
			Serial.print("menu mode ");
			Serial.println(menu_mode);
			debounce_set = false;
		}
	}
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
bool grblStatusEvaluation(uint8_t *buf, int count)
{
	bool rs = false;
	uint8_t *start_pos = (uint8_t*)memchr(buf, '<', count);
	if(start_pos)												// Status GRBL
	{
		uint8_t *end_pos = (uint8_t*)memchr(buf, '>', count);
		if(end_pos) {
			memcpy(status_process_buf, start_pos+1, end_pos - start_pos - 1);
			status_process_buf[end_pos - start_pos - 1] = 0;
DBGSerial.println(status_process_buf);
			char *p1 = status_process_buf;
			char *p2 = strchr(p1, '|');
			if(p2) {
				rs = true;
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
					} else if(strncmp(p1, "Pn:", 4) == 0) {
						// work position
						p1 += 3;
						status.limit_state = p1;
					}
				}
			}
		}
	}
	return rs;
}
