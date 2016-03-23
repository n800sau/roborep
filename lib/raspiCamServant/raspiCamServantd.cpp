#include "raspiCamServant.h"
#include <stdio.h>

int main(int argc, char const **argv)
{
	raspiCamServant srv = raspiCamServant((argc > 1) ? argv[1] : NULL);
	srv.create_servant();
//	srv.setLoopInterval(10);
	srv.run();
	return 0;
}
