#include "misc_utils.h"

void print_ts(Print &printer=Serial)
{
	struct tm timeinfo;
	if(getLocalTime(&timeinfo)) {
		printer.print(&timeinfo, TS_FORMAT);
	}
}

void print_ts_prefix(Print &printer=Serial)
{
	printer.print("[");
	print_ts(printer);
	printer.print("]");
}
