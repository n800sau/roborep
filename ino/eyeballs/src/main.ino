#include <Servo.h>

#define STOP true

// eye            s1          eye
// s2 bottom     s3 middle    s5 bottom
// s4 top                     s6 top


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
	{8, 60, 100, false},
	{9, 40, 100, false}
};

void setup()
{
	for(int i=0; i<SCOUNT; i++)
	{
		s[i].middle_val = s[i].min_val + (s[i].max_val-s[i].min_val)/2;
		s[i].s.attach(s[i].pin);
		s[i].s.write(s[i].middle_val);
	}
}

void loop()
{
	if(!STOP) {
		for(int i=0; i<SCOUNT; i++)
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
	}
}
