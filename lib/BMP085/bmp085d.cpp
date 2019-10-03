#include "BMP085.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	BMP085 srv = BMP085();
	srv.run();
	return 0;
}
