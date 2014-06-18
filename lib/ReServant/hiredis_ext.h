#ifndef __HIREDIS_EXT_H

#define __HIREDIS_EXT_H

#include <stdio.h>
#include <hiredis.h>
#include <async.h>

void *redisCommandN(redisContext *c, int num, ...);
int redisAsyncCommandN(redisAsyncContext *ac, redisCallbackFn *fn, void *privdata, int num, ...);

void fprint_reply(redisReply *reply, int indent=0, FILE *stream=stdout);

#endif
