#include "l3g4200d_proc.h"

L3G4200D gyro;

void setup_gyro()
{
//	gyro.enableDefault();
	gyro.enableDefault(250);
}

void process_gyro()
{
	gyro.read();
}

