#include "tcpserver.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	TCPServer srv = TCPServer(7880);
	srv.run();
	return 0;
}
