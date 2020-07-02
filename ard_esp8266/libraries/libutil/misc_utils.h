#ifndef __MISC_UTILS_H
#define __MISC_UTILS_H

#include <Arduino.h>

#define TS_FORMAT "%Y-%m-%d %H:%M:%S"

bool getLocalTime(struct tm * info, uint32_t ms=5000);

void print_ts(Print &printer=Serial, const char *format=NULL);

void print_ts_prefix(Print &printer=Serial, const char *format=NULL);

void goto_deepsleep(int secs);

#endif // __MISC_UTILS_H
