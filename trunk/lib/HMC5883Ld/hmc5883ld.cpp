#include <stdio.h>
#include "HMC5883L.h"

int main()
{
	HMC5883L srv = HMC5883L(); // Construct a new HMC5883 compass
	srv.run();
	return 0;
}
