#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include "hiredis_ext.h"

int redisAsyncCommandN(redisAsyncContext *ac, redisCallbackFn *fn, void *privdata, int num, ...)
{
	const char **argv=(const char **)calloc(num, sizeof(const char *));
	size_t *argl = (size_t *)calloc(num, sizeof(size_t));
	int i;
	va_list vl;
	va_start(vl, num);
	for(i=0; i < num; i++)
	{
		argv[i] = va_arg(vl, const char *);
		argl[i] = strlen(argv[i]);
	}
	va_end(vl);
	int rs = redisAsyncCommandArgv(ac, fn, privdata, num, argv, argl);
	free(argv);
	free(argl);
	return rs;
}

void *redisCommandN(redisContext *c, int num, ...)
{
	const char **argv=(const char **)calloc(num, sizeof(const char *));
	size_t *argl = (size_t *)calloc(num, sizeof(size_t));
	int i;
	va_list vl;
	va_start(vl, num);
	for(i=0; i < num; i++)
	{
		argv[i] = va_arg(vl, const char *);
		argl[i] = strlen(argv[i]);
	}
	va_end(vl);
	void *rs = redisCommandArgv(c, num, argv, argl);
	free(argv);
	free(argl);
	return rs;
}
