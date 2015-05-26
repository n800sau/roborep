#include "cam_service.h"
#include "uart_utils.h"

CMUcam4::CMUcam4()
{
	version[0] = 0;
}

bool CMUcam4::send_request(const char *request, void (*cb)(void *), void *arg, int ms)
{
	Serial1.println("here1");
	bool rs = !uartIsLocked();
	if(rs) {
		Serial1.println("here2");
		uartLock();
		Serial1.println("here3");
		uart_state = uartSetPrimary(false);
		Serial1.println("here4");
		uart_baud = uartBegin(19200);
		Serial1.println("here5");
		repbufpos = 0;
		Serial.write("\r");
		Serial.write(request);
		Serial.write("\r");
		Serial1.println("here6");
		if(cb) {
			ms_start = millis();
			ticker.attach_ms(ms, cb, arg);
		}
	}
	return rs;
}

void CMUcam4::end_request()
{
	ticker.detach();
	uartSetPrimary(uart_state);
	uartBegin(uart_baud);
	uartUnlock();
}

bool CMUcam4::check_reply(char *buffer, size_t bufsize)
{
	bool rs = false;
	while(Serial.available() > 0) {
		char b = Serial.read();
		if( b == '\r' ) {
			int sz = min(bufsize, repbufpos);
			memcpy(buffer, repbuf, sz);
			buffer[sz] = 0;
			repbufpos = 0;
			rs = true;
			break;
		} else {
			repbuf[repbufpos++] = b;
			if(repbufpos >= REPBUF_SIZE) {
				for(int i=0; i<REPBUF_SIZE-1; i++) {
					repbuf[i] = repbuf[i+1];
				}
				repbufpos--;
			}
		}
	}
	return rs;
}

void CMUcam4::readVersionEnd(void *arg)
{
	CMUcam4 *self = (CMUcam4 *)arg;
	Serial1.println("read version end");
	if(self->check_reply(self->version, sizeof(self->version))) {
		Serial1.print("Version:");
		Serial1.println(self->version);
		self->end_request();
	} else if(millis() - self->ms_start >= 1000) {
		Serial1.println("timeout");
		self->end_request();
	}
}

bool CMUcam4::readVersion()
{
	return send_request("GV", readVersionEnd, this);
}

CMUcam4 cam;

void CMUcam4::begin()
{

	Serial1.println("cam begin");
	cam.readVersion();
}

void ICACHE_FLASH_ATTR setupCAMservice()
{
	cam.begin();
}

void ICACHE_FLASH_ATTR handleCAMservice()
{
	if(!cam.readVersion()) {
		Serial1.println("locked");
	}
}
