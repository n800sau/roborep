#include "rhttp.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	Rhttp srv = Rhttp(7880);
	srv.run();
	return 0;
}
