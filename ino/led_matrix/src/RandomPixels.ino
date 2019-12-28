#include <avr/sleep.h>
#include <SparkFun_LED_8x7.h>
#include <Chaplex.h>

#define Output Serial
#define N_ROWS 7
#define N_COLS 8

#define BTN_PIN 2

// Global variables
byte led_pins[] = {4, 5, 6, 7, 8, 9, 10, 11}; // Pins for LEDs

int row = 0, col = 0;
bool running = false;


void wakeUp()
{
	sleep_disable();
	detachInterrupt(0);
	digitalWrite(LED_BUILTIN, HIGH);
	running = true;
}

void gotoSleep()
{
	sleep_enable();
	attachInterrupt(0, wakeUp, LOW);
	digitalWrite(LED_BUILTIN, LOW);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void setup()
{
	Output.begin(115200);
	pinMode(BTN_PIN, INPUT_PULLUP);
	pinMode(LED_BUILTIN, INPUT_PULLUP);

	// Initialize and clear display
	Plex.init(led_pins);
	Plex.clear();
	Plex.display();
	gotoSleep();
}

void loop()
{
	if(running) {
		// Write to the LED display and wait before doing it again
		Plex.pixel(col, row, HIGH);
		Plex.display();
		Output.print(row);
		Output.print(",");
		Output.println(col);
		if(++col >= N_COLS) {
			col = 0;
			if(++row == 3) {
				row++;
			}
			if(row >= N_ROWS) {
				row = 0;
				for(int c=0; c<N_COLS; c++) {
					Plex.pixel(c, 3, HIGH);
				}
				Plex.display();
				running = false;
				delay(3000);
				Plex.clear();
				Plex.display();
				gotoSleep();
			}
		}
		delay(100);
	} else {
		if(digitalRead(BTN_PIN) == LOW) {
			running = true;
		}
	}
}