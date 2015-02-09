// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <stdlib.h>

#define MAX_SPEED 100

const char instructions_line1[] PROGMEM = "Searchi";
const char instructions_line2[] PROGMEM = "ng...  ";

// Displays the battery voltage.
int bat_test(int max_val)
{
	int bat = read_battery_millivolts();

	lcd_goto_xy(0,0);
	print_long(bat);
	print("mV");
	lcd_goto_xy(0,1);
	print_long(max_val);
	print("mV");
	int enc1 = encoders_get_counts_m1();
	lcd_goto_xy(0,2);
	print_long(enc1);
	int enc2 = encoders_get_counts_m2();
	lcd_goto_xy(0,3);
	print_long(enc2);

	return bat;
}

/*
  Read random seed from eeprom and write a new random one.
*/
void initrand()
{
	uint32_t state;
	static uint32_t EEMEM sstate;

	state = eeprom_read_dword(&sstate);

	// Check if it's unwritten EEPROM (first time). Use something funny
	// in that case.
	if (state == 0xffffffUL)
		state = 0xDEADBEEFUL;
	srandom(state);
	eeprom_write_dword(&sstate, random());
}

void initialize()
{
	// This must be called at the beginning of 3pi code, to set up the
	// sensors.  We use a value of 2000 for the timeout, which
	// corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
	initrand();
	pololu_3pi_init(2000);

	clear();
	print_from_program_space(instructions_line1);
	lcd_goto_xy(0,1);
	print_from_program_space(instructions_line2);
	delay_ms(1000);

}

int8_t m1_back = 0, m2_back = 0;
int m1_speed, m2_speed;

void start_random_move()
{
	m1_back = rand() % 2;
	m2_back = rand() % 2;
	m1_speed = rand() % MAX_SPEED;
	m2_speed = rand() % MAX_SPEED;
	set_motors(m1_speed * (m1_back ? -1 : 1), m2_speed * (m2_back ? -1 : 1));
}

void stop_move()
{
	set_motors(0, 0);
}

int main()
{
	int i;

	// set up the 3pi
	initialize();

	// Move to random place
	start_random_move();
	delay_ms(1000);
	stop_move();
	delay_ms(500);
	// stopped
	encoders_init(0, 0, 0, 0);

	// This is the "main loop" - it will run forever.
	int max_val = 0;
	int val = bat_test(max_val);
	// start moving
	start_random_move();

	for(i=0; i<100; i++)
	{
		// get bat value
		val = bat_test(max_val);
		if(val > max_val) {
			// closer to the power point
			max_val = val;
			// keep on moving
		} else {
			// further from the power point
			// change direction
			start_random_move();
		}


		delay_ms(100);
	}
}
