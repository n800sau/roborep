#include "LSM303.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	LSM303 srv = LSM303();
	srv.run();
	return 0;
}
