#ifndef __HIREDIS_EXT_H

#define __HIREDIS_EXT_H

#include <hiredis.h>
#include <async.h>

void *redisCommandN(redisContext *c, int num, ...);
int redisAsyncCommandN(redisAsyncContext *ac, redisCallbackFn *fn, void *privdata, int num, ...);

#endif
