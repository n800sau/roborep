/*

am2320
sda - A4
scl - A5

heater pwm - 3

*/

#include <AM2320.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ticker.h>

#define OLED_ADDRESS 0x3c
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);


AM2320 am2320;

const int HEATER_PIN = 3;

const int MAX_HEAT_VAL = 100;

int heat_val = MAX_HEAT_VAL;

int temp2set = 37;

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=150, Ki=1, Kd=100;
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;

void display_status()
{
	if(temp != UNKNOWN_TEMP) {
		display.clearDisplay();
		display.setCursor(0, 0);
		display.print(temp);
		display.print(F(" => "));
		display.print(temp2set);
		display.print(F(" C"));

		Serial.print("\n\nHeating val: ");
		Serial.print(heat_val);
		if(heat_val > 0) {
			display.fillTriangle(2, 12, 12, 2, 22, 12, INVERSE);
		}
		Serial.print(", ");
		Serial.print(temp);
		Serial.print(F(" > "));
		Serial.print(temp2set);
		Serial.print(", Humidity: ");
		Serial.println(am2320.getHumidity());
		display.display();
	}
}

Ticker status_timer(display_status, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	am2320.begin();
	pinMode(HEATER_PIN, OUTPUT);
	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { // Address 0x3D for 128x64
		Serial.println(F("SSD1306 allocation failed"));
	} else {
		display.clearDisplay();
		display.display();
		// text display tests
		display.setTextSize(1);
		display.setTextColor(WHITE);
		display.setCursor(30, 10);
		display.print(F("I an a heater"));
		display.display();
		status_timer.start();
	}
}
void loop()
{
	if (am2320.measure()) {
		temp = am2320.getTemperature();
		Input = temp;
		Setpoint = temp2set;
		heaterPID.Compute();
		heat_val = Output < 0 ? 0 : (Output > MAX_HEAT_VAL ? MAX_HEAT_VAL : Output);
		analogWrite(HEATER_PIN, heat_val);
	}
	else
	{
		analogWrite(HEATER_PIN, 0);
		int errorCode = am2320.getErrorCode();
		switch (errorCode)
		{
			case 1: Serial.println("ERR: am2320 is offline"); break;
			case 2: Serial.println("ERR: CRC validation failed."); break;
		}
	}
	status_timer.update();
}
