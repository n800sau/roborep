/**************************************************************************/
#ifndef CMD_H
#define CMD_H

#define MAX_MSG_SIZE    60
#include <stdint.h>

void cmdInit(Stream *);
void cmdPoll();
void cmdAdd(const char *name, void (*func)(int argc, char **argv));
Stream* cmdGetStream(void);

#endif //CMD_H
