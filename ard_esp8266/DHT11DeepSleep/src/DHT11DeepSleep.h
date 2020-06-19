#include <c_types.h>

struct DATA_T {
	float temp_c;
	float humidity;
	float v;
	unsigned long timestamp;
};

#define DATA_MAGIC uint8_t(0x34)

struct HEADER_T {
	uint8_t magic;
	int16_t send_first;
	int16_t send_last;
};


