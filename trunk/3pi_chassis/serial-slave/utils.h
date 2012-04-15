#ifndef __UTILS_H

#define __UTILS_H

#define MAXVAL 0x1FFF
#define MINVAL -0x1FFF

short bytes2short(char lb, char hb);

//bytes - char bytes[2];
void short2bytes(short val, char *bytes);

#endif //__UTILS_H
