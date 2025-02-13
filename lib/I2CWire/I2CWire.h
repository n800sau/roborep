#ifndef I2CWire_h
#define I2CWire_h

#include <stdint.h>

class I2CWire
{

	protected:
		int fd;
	public:
		I2CWire();
		void selectDevice(uint8_t addr, const char * name);
		int requestFromDevice(uint8_t addr, int length, uint8_t buf[]);
		int requestFromDeviceLH(uint8_t addr, int length, uint16_t buf[]);
		int requestFromDeviceHL(uint8_t addr, int length, uint16_t buf[]);
		bool writeToDevice(uint8_t reg, uint8_t val);
};

#endif //I2CWire
