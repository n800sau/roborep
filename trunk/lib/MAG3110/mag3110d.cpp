#include "MAG3110.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	MAG3110 srv = MAG3110();
	srv.run();
	return 0;
}
