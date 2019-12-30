#include "uart_utils.h"

bool uart_is_primary = true;
bool uart_locked = false;

int uart_baudrate = 0;

int uartBegin(int baudrate)
{
	int rs = uart_baudrate;
	if(baudrate > 0) {
		if(uart_baudrate > 0) {
			Serial1.println("here6");
			Serial.end();
			Serial1.println("here7");
		}
		uart_baudrate = baudrate;
		Serial1.println("here8");
		Serial.begin(uart_baudrate);
		Serial1.println("here9");
	}
	return rs;
}

bool ICACHE_FLASH_ATTR uartIsPrimary()
{
	return uart_is_primary;
}

bool ICACHE_FLASH_ATTR uartSetPrimary(bool prim)
{
	bool old_state = uart_is_primary;
	if(prim != uart_is_primary) {
		Serial.swap();
		uart_is_primary = prim;
	}
	return old_state;
}

// return true if locked and available
// return false if fail
bool uartLock()
{
	bool rs = !uart_locked;
	if(rs) {
		uart_locked = true;
	}
	return rs;
}

bool uartUnlock()
{
	uart_locked = false;
}

bool uartIsLocked()
{
	return uart_locked;
}
