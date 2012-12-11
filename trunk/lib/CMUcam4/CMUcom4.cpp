#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include "CMUcom4.h"
#include <wiringSerial.h>


const char *serial_devices[] = {
	"/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
	"/dev/ttyUSB0",
	"/dev/ttyAMA0"
};
#define N_SERIALS (sizeof(serial_devices) / sizeof(*serial_devices))

/*******************************************************************************
* Constructor Functions
*******************************************************************************/

CMUcom4::CMUcom4():_millis(time(NULL))
{
    _port = CMUCOM4_SERIAL;
}

CMUcom4::CMUcom4(int port):_millis(time(NULL))
{
    _port = port;
}

/*******************************************************************************
* Public Functions
*******************************************************************************/

void CMUcom4::begin(unsigned long baud)
{
//	printf("open %d\n", baud);
	_fd = serialOpen((char*)serial_devices[(_port >= 0 && _port < N_SERIALS) ? _port : 0], baud);
}

void CMUcom4::end()
{
	serialClose(_fd);
}

int CMUcom4::read()
{
	return serialGetchar(_fd);
}

size_t CMUcom4::write(uint8_t c)
{
//	printf("write1 '%c'\n", c);
	serialPutchar(_fd, c);
	return 1;
}

size_t CMUcom4::write(const char * str)
{
//	printf("write2 \n%s\n", str);
	serialPuts(_fd, (char*)str);
	return strlen(str);
}

size_t CMUcom4::write(const uint8_t * buffer, size_t size)
{
//	printf("write3 \n%s\n", buffer);
	size_t rs=0;
	for(int i=0; i< size; i++) {
		rs += write(buffer[i]);
	}
	return rs;
}

int CMUcom4::available()
{
	return serialDataAvail(_fd);
}

void CMUcom4::flush()
{
	serialFlush(_fd);
}

void CMUcom4::delayMilliseconds(unsigned long ms)
{
    usleep(ms * 1000);
}

unsigned long CMUcom4::milliseconds()
{
    return time(NULL) - _millis;
}

