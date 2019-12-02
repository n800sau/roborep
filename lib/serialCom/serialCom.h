#ifndef _SERIALCOM_H_
#define _SERIALCOM_H_
 
#ifdef __cplusplus
extern "C"
{
#endif
 
#include <termios.h>
 
  /* Controls :  CS8      = 8 bits
   *             CREAD    = Enable reading */
#define SERIAL_CONTROL  (CS8 | CREAD)
#define NO_PARITY_CHECK 0
#define PARITY_CHECK    1
 
  /* Input    :  PARMRK   = If IGNPAR is not set, prefix a character with a parity error  or
   framing  error  with  \377  \0 */
#define SERIAL_INPUT    PARMRK
 
  /* Handler type definition */
  typedef void
  (*serial_handler)(int status);
 
  /* Serial port initialization function */
  int
  serialConfiguration(serial_handler, const char* device, tcflag_t baudrate,
      char parity);
 
#ifdef __cplusplus
}
#endif
 
#endif
