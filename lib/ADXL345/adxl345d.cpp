#include "ADXL345.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	ADXL345 srv = ADXL345();
	srv.setRange(2, true);
	srv.enableMeasurements();
	srv.create_servant();
	srv.setLoopInterval(0.1);
	srv.run();
	return 0;
}
