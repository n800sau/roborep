#ifndef __CAM_SERVICE_H

#define __CAM_SERVICE_H

#include <c_types.h>
#include <Ticker.h>

#define REPBUF_SIZE 256

class CMUcam4
{
	protected:
		char repbuf[REPBUF_SIZE];
		int repbufpos;
		int uart_state;
		int uart_baud;
		Ticker ticker;
		static void readVersionEnd(void *self);
		char version[20];
		unsigned long ms_start;
	public:
		CMUcam4();
		void begin();
		bool send_request(const char *request, void (*callback)(void *), void *arg, int ms=100);
		void end_request();
		bool check_reply(char *buffer, size_t bufsize);
		bool readVersion();
};


void ICACHE_FLASH_ATTR setupCAMservice();

void ICACHE_FLASH_ATTR handleCAMservice();

#endif //__CAM_SERVICE_H
