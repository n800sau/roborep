#include <stdio.h>
#include "HMC5883L.h"

int main()
{
	HMC5883L srv = HMC5883L(); // Construct a new HMC5883 compass
	srv.create_servant();
	srv.setLoopInterval(1);
	srv.run();
	return 0;
}
