#include <Servo.h>

#define STOP false

// eye left           s1 (2)         eye right
// s2(1) bottom     s3(0) middle    s5 bottom (4)
// s4(3) top                     s6 top (5)


#define SCOUNT 6

struct SRV {
	const int pin;
	const int min_val;
	const int max_val;
	int middle_val;
	bool reverse;
	Servo s;
} s[SCOUNT] ={
	{4, 40, 100, false},
	{5, 50, 90, false},
	{6, 60, 100, false},
	{7, 80, 120, false},
	{8, 100, 60, false},
	{9, 40, 100, false}
};

void setup()
{
	all_attach();
	delay(2000);
}

void open_left_eye()
{
	s[1].s.write(s[1].min_val);
	s[3].s.write(s[3].min_val);
}

void close_left_eye()
{
	s[1].s.write(s[1].max_val);
	s[3].s.write(s[3].max_val);
}

void open_right_eye()
{
	s[4].s.write(s[4].min_val);
	s[5].s.write(s[5].min_val);
}

void close_right_eye()
{
	s[4].s.write(s[4].max_val);
	s[5].s.write(s[5].max_val);
}

void look_left()
{
	s[0].s.write(s[0].max_val);
}

void look_right()
{
	s[0].s.write(s[0].min_val);
}

void look_straight()
{
	s[0].s.write(s[0].middle_val);
}

void look_up()
{
	s[2].s.write(s[2].max_val);
}

void look_down()
{
	s[2].s.write(s[2].min_val);
}

void look_middle()
{
	s[2].s.write(s[2].middle_val);
}

void all_attach()
{
	for(int i=0; i<SCOUNT; i++)
	{
		s[i].middle_val = s[i].min_val + (s[i].max_val-s[i].min_val)/2;
		s[i].s.attach(s[i].pin);
		s[i].s.write(s[i].middle_val);
	}
}

void all_detach()
{
	for(int i=0; i<SCOUNT; i++)
	{
		s[i].s.detach();
	}
}

void loop()
{
	if(!STOP) {
		all_attach();
		look_up();
		delay(1000);
		look_left();
		delay(1000);
		open_left_eye();
		delay(1000);
		close_left_eye();
		delay(1000);
		look_down();
		delay(1000);
		look_right();
		delay(1000);
		open_right_eye();
		delay(1000);
		close_right_eye();
		delay(1000);
		look_straight();
		delay(1000);
		look_middle();
		delay(1000);
/*
		for(int i=4; i<5; i++)
		{
			for(int v=s[i].middle_val; v<s[i].max_val; v++) {
				s[i].s.write(v);
				delay(15);
			}
			for(int v=s[i].max_val; v>=s[i].min_val; v--) {
				s[i].s.write(v);
				delay(15);
			}
			for(int v=s[i].min_val; v<s[i].middle_val; v++) {
				s[i].s.write(v);
				delay(15);
			}
		}
*/
		all_detach();
		delay(5000);
	}
}
