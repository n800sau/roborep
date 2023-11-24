#include "HD44780.h"
#include <stdio.h>
#include <libgen.h>
#include <linux/limits.h>

int main(int argc, char const **argv)
{
	HD44780 srv = HD44780();
	char *dirc = strdup(argv[0]);
	char actualpath[PATH_MAX+1];
	realpath(dirname(dirc), actualpath);
	strcat(actualpath, "/i.lst");
//	printf("%s\n", actualpath);
	srv.setListPath(actualpath);
	free((void*) dirc);
	srv.run();
	return 0;
}
