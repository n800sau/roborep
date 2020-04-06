#include "userver.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	UServer srv = UServer(7880);
	srv.run();
	return 0;
}
