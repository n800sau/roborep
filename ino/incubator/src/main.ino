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
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);


AM2320 am2320;


#if defined(ESP8266)
const int HEATER_PIN = 4;
const int HEATING_LED_PIN = 5;
const int FAN_PIN = 11;
#else // arduino
const int HEATER_PIN = 3;
const int HEATING_LED_PIN = 4;
const int FAN_PIN = 5;
#endif

// 80 - 8v
const int MAX_HEAT_PWM = 80;

int heat_val = MAX_HEAT_PWM;

int temp2set = 37;

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=80, Ki=0, Kd=10;
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int UNKNOWN_TEMP = -10000;
float temp = UNKNOWN_TEMP;

void display_status()
{
	if(temp != UNKNOWN_TEMP) {
		display.clearDisplay();
		display.setCursor(18, 0);
		display.print("T: ");
		display.print(temp);
		display.print(F(" => "));
		display.print(temp2set);
		display.print(F(" C"));
		display.setCursor(18, 11);
		display.print("Heating: ");
		display.print(heat_val);
		if(heat_val > 0) {
			display.fillTriangle(2, 18, 10, 18, 6, 10, INVERSE);
		}

		Serial.print("\n\nHeating val: ");
		Serial.print(heat_val);
		Serial.print(", ");
		Serial.print(temp);
		Serial.print(F(" > "));
		Serial.print(temp2set);
		Serial.print(", Humidity: ");
		Serial.println(am2320.getHumidity());
		display.display();
	}
}

#if defined(ESP8266)
Ticker status_timer;
#else
Ticker status_timer(display_status, 1000, 0, MILLIS);
#endif

void setup()
{
	Serial.begin(115200);
	Serial.println("Incubator...");
	am2320.begin();
	pinMode(HEATER_PIN, OUTPUT);
	pinMode(FAN_PIN, OUTPUT);
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

		//tell the PID to range between 0 and max pwm
		heaterPID.SetOutputLimits(0, MAX_HEAT_PWM);

		//turn the PID on
		heaterPID.SetMode(AUTOMATIC);
		heaterPID.SetSampleTime(10);

#if defined(ESP8266)
		status_timer.attach_ms(1000, display_status);
#else
		status_timer.start();
#endif
	}
}
void loop()
{
	if (am2320.measure()) {
		temp = am2320.getTemperature();
		Input = temp;
		Setpoint = temp2set;
		heaterPID.Compute();
		Serial.print("In:");
		Serial.print(Input);
		Serial.print(" Out:");
		Serial.print(Output);
		Serial.print(" Set:");
		Serial.println(Setpoint);
		heat_val = Output;
		heat_val = MAX_HEAT_PWM;
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
#if !defined(ESP8266)
	status_timer.update();
#endif
	delay(500);
}
