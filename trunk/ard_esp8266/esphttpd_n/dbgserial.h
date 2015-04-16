#ifndef __DBG_SERIAL_H
#define __DBG_SERIAL_H

#include <HardwareSerial.h>

typedef struct callback_write {
	void (*function)(uint8_t c);
} callback_write_t;

class DbgSerial: public HardwareSerial {

	protected:
		callback_write_t cbproc;

	public:

		DbgSerial(UARTnr_t uart_nr, callback_write_t cb);
		size_t write(uint8_t) override;

};

extern DbgSerial dbgSerial;

#endif //__DBG_SERIAL_H
