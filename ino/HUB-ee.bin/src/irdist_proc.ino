#include "irdist_proc.h"

int IRpin = A3;
float distance = -1;

void setup_irdist()
{
}

void process_irdist()
{
	float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
	distance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)
}

