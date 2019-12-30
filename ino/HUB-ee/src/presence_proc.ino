#include "presence_proc.h"

int MVcount = 0;
static int MVpin = A2;
static bool lastMVfound = false;

void setup_presence()
{
}

void process_presence()
{
	bool MVfound = analogRead(MVpin) > 300;
	if(MVfound != lastMVfound) {
		lastMVfound = MVfound;
		if(MVfound) {
			MVcount += 1;
		}
	}
}

