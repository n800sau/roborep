#include "ADXL345.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	ADXL345 srv = ADXL345(7880);
	srv.setRange(2, true);
	srv.enableMeasurements();
	srv.run();
	return 0;
}
