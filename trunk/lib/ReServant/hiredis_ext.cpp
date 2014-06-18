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

void fprint_reply(redisReply *reply, int indent, FILE *stream)
{
	int i;
	for(i=0; i<indent; i++) {
		putc(' ', stream);
	}
	if(reply) {
		switch(reply->type) {
			case REDIS_REPLY_STRING:
				fprintf(stream, "(string) %s\n", reply->str);
				break;
			case REDIS_REPLY_ARRAY:
				for(i=0; i<reply->elements; i++) {
					fprint_reply(reply->element[i], indent+4, stream);
				}
				break;
			case REDIS_REPLY_INTEGER:
				fprintf(stream, "(integer) %lld\n", reply->integer);
				break;
			case REDIS_REPLY_NIL:
				fprintf(stream, "(nil reply)\n");
				break;
			case REDIS_REPLY_STATUS:
				fprintf(stream, "(status) %s\n", reply->str);
				break;
			case REDIS_REPLY_ERROR:
				fprintf(stream, "(error) %s\n", reply->str);
				break;
			default:
				fprintf(stream, "(unknown type %d)\n", reply->type);
				break;
		}
	} else {
		fprintf(stream, "%s\n", reply);
	}
}
