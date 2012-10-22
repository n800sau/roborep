#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include "CMUcom4.h"
#include <wiringSerial.h>


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
	_fd = serialOpen((char*)"/dev/ttyAMA0", baud);
}

void CMUcom4::end()
{
	serialClose(_fd);
}

int CMUcom4::read()
{
	serialGetchar(_fd);
}

size_t CMUcom4::write(uint8_t c)
{
	serialPutchar(_fd, c);
	return 1;
}

size_t CMUcom4::write(const char * str)
{
	serialPuts(_fd, (char*)str);
	return strlen(str);
}

size_t CMUcom4::write(const uint8_t * buffer, size_t size)
{
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

int CMUcom4::peek()
{
}

void CMUcom4::delayMilliseconds(unsigned long ms)
{
    usleep(ms * 1000);
}

unsigned long CMUcom4::milliseconds()
{
    return time(NULL) - _millis;
}
