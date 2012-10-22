#include <unistd.h>
#include <time.h>
#include "CMUcom4.h"

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
}

void CMUcom4::end()
{
}

int CMUcom4::read()
{
}

size_t CMUcom4::write(uint8_t c)
{
}

size_t CMUcom4::write(const char * str)
{
}

size_t CMUcom4::write(const uint8_t * buffer, size_t size)
{
}

int CMUcom4::available()
{
}

void CMUcom4::flush()
{
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
