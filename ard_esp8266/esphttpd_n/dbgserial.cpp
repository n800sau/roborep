#include "dbgserial.h"

DbgSerial::DbgSerial(UARTnr_t uart_nr, callback_write_t cb):
	HardwareSerial(uart_nr),cbproc(cb)
{
}

size_t DbgSerial::write(uint8_t c)
{
	size_t rs = HardwareSerial::write(c);
	cbproc.function(c);
	return rs;
}
