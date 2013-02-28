#include "kalman.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	Kalman k = Kalman();
	k.run();
	return 0;
}
