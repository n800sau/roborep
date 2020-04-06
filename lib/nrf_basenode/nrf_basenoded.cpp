#include "nrf_basenode.h"
#include <stdio.h>
#include <unistd.h>

void usage(const char *prog)
{
	printf("Usage: %s -f <data file name>\n", prog);
	exit(1);
}

int main(int argc, char *const *argv)
{
	int ch;
	char *datafname = NULL;
	while((ch = getopt(argc, argv, "f:h")) != -1)
	{
		switch(ch)
		{
			case 'f':
				datafname = optarg;
				break;
			case 'h':
				usage(argv[0]);
				break;
			default:
				usage(argv[0]);
				break;
		}
	}
	NRF_BASENODE srv = NRF_BASENODE(datafname);
	srv.run();
	return 0;
}
