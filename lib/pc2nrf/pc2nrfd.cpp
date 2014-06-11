#include "pc2nrf.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	PC2NRF srv = PC2NRF();
	srv.run();
	return 0;
}
