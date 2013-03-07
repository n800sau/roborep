#include "MPU6050.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	MPU6050 srv = MPU6050();
	srv.run();
	return 0;
}
