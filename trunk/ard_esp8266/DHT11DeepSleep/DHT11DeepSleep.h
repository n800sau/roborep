struct DATA_T {
	bool sent; // if the data block has been sent
	float temp_c;
	float humidity;
	float v;
	unsigned long timestamp;
};

#define DATA_T_MAGIC uint8_t(0x34)
