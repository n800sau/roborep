#include "ROStf.h"

int main(int argc, char **argv)
{
	ROStf srv = ROStf(argc, argv);
	srv.run();
	return 0;
}

