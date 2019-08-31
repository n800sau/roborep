#include "L3G4200D.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	L3G4200D gyro = L3G4200D();
	gyro.run();
	return 0;
}
