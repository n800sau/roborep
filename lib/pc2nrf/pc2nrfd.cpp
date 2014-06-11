#include "pc2nrf.h"
#include <iostream>
#include <exception>
#include <errno.h>

int main(int argc, char const **argv)
{
	try {
		PC2NRF srv = PC2NRF();
		srv.run();
		return 0;
	} catch (const std::exception& e) {
		std::cerr << "Error " << e.what() << endl << strerror(errno) << endl;
		return 1;
	}
}
