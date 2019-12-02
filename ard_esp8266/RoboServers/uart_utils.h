#ifndef __T2S_SERVICE_H

#define __T2S_SERVICE_H

#include <c_types.h>

// returns old baud rate
int uartBegin(int baudrate);

bool ICACHE_FLASH_ATTR uartSetPrimary(bool prim);
bool ICACHE_FLASH_ATTR uartIsPrimary();

// return true if locked and available
// return false if fail
bool uartLock();
bool uartUnlock();
bool uartIsLocked();

#endif //__T2S_SERVICE_H
