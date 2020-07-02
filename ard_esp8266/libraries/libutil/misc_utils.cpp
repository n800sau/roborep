#include "misc_utils.h"

bool getLocalTime(struct tm * info, uint32_t ms)
{
  uint32_t count = ms / 10;
  time_t now;

  time(&now);
  localtime_r(&now, info);

  if (info->tm_year > (2016 - 1900)) {
    return true;
  }

  while (count--) {
    delay(10);
    time(&now);
    localtime_r(&now, info);
    if (info->tm_year > (2016 - 1900)) {
      return true;
    }
  }
  return false;
}

void print_ts(Print &printer, const char *format)
{
	struct tm timeinfo;
	if(getLocalTime(&timeinfo)) {
		char buf[64];
		if(strftime(buf, sizeof(buf), format ? format : TS_FORMAT, &timeinfo) > 0){
			printer.print(buf);
		}
	}
}

void print_ts_prefix(Print &printer, const char *format)
{
	printer.print("[");
	print_ts(printer, format);
	printer.print("]");
}

void goto_deepsleep(int secs)
{
	Serial.flush(); // to let serial finish its output
	ESP.deepSleep(secs * 1000000L, WAKE_RF_DEFAULT);
	// will reboot, GPIO16 must be connected to reset
}

