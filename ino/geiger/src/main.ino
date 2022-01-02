#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

#define PIN_COUNTER 2
#define PIN_BRIGHTNESS 10

// This Sketch counts the number of pulses a minute.
// Connect the GND on Arduino to the GND on the Geiger counter.
// Connect the 5V on Arduino to the 5V on the Geiger counter.
// Connect the VIN on the Geiger counter to the D2 on Arduino.

#define STEP_PERIOD 5000
#define STEP_COUNT 12

volatile unsigned long counts[STEP_COUNT]; //variable for GM Tube events
volatile int cur_step;
bool full_data;

unsigned long previousMillis; //variable for measuring time

void impulse()
{
	counts[cur_step]++;
}

void setup()
{
	full_data = false;
	for(int i=0; i<STEP_COUNT; i++) {
		counts[i] = -1;
	}
	cur_step = 0;
	Serial.begin(115200);
	pinMode(PIN_COUNTER, INPUT);
	pinMode(PIN_BRIGHTNESS, OUTPUT);
	analogWrite(PIN_BRIGHTNESS, 70);
	lcd.begin(16, 2);
	attachInterrupt(digitalPinToInterrupt(PIN_COUNTER), impulse, FALLING); //define external interrupts
	Serial.println("Start counter");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Counter");
	show_step();
}

char step_marker[STEP_COUNT+1];
void show_step()
{
	for(int i=0; i< STEP_COUNT; i++) {
		step_marker[i] = i == cur_step ? '*' : ' ';
	}
	step_marker[STEP_COUNT] = 0;
	lcd.setCursor(0, 1);
	lcd.print(step_marker);
}

void loop()
{
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis > STEP_PERIOD)
	{
		if(cur_step == STEP_COUNT-1)
		{
			full_data = true;
		}
		if(full_data) {
			int tot_counts = 0;
			int i_step = cur_step;
			for(int i=0; i<STEP_COUNT; i++)
			{
				tot_counts += counts[i_step];
				i_step--;
				if(i_step < 0)
				{
					i_step = STEP_COUNT - 1;
				}
			}
			Serial.println(tot_counts);
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(tot_counts);
			lcd.setCursor(6, 0);
			lcd.print("count/min");
		}
		previousMillis = currentMillis;
		cur_step++;
		if(cur_step >= STEP_COUNT)
		{
			cur_step = 0;
		}
		counts[cur_step] = 0;
		show_step();
	}
}
