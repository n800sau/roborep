#ifndef _CMUCOM4_H_
#define _CMUCOM4_H_

#include <stdlib.h>
#include <stdint.h>

// Try to save RAM for non-Mega boards.
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CMUCOM4_INPUT_BUFFER_SIZE   256 ///< Responce input buffer size.
#define CMUCOM4_OUTPUT_BUFFER_SIZE  256 ///< Command output buffer size.
#else
#define CMUCOM4_INPUT_BUFFER_SIZE   160 ///< Responce input buffer size.
#define CMUCOM4_OUTPUT_BUFFER_SIZE  96 ///< Command output buffer size.
#endif

/***************************************************************************//**
* This function macro expands whatever argument name that was passed to this
* function macro into a string. @par For example:
* <tt>@#define ARDUINO 100</tt> @n
* <tt>%CMUCOM4_N_TO_S(ARDUINO)</tt> exapands to @c "ARDUINO"
*******************************************************************************/
#define CMUCOM4_N_TO_S(x)           #x

/***************************************************************************//**
* This function macro expands whatever argument value that was passed to this
* function macro into a string. @par For example:
* <tt>@#define ARDUINO 100</tt> @n
* <tt>%CMUCOM4_V_TO_S(ARDUINO)</tt> exapands to @c "100"
*******************************************************************************/
#define CMUCOM4_V_TO_S(x)           CMUCOM4_N_TO_S(x)

/***************************************************************************//**
* Default firmware startup baud rate number.
*******************************************************************************/
#define CMUCOM4_SLOW_BAUD_RATE      19200

/***************************************************************************//**
* Default firmware startup baud rate string.
*******************************************************************************/
#define CMUCOM4_SLOW_BR_STRING      CMUCOM4_V_TO_S(CMUCOM4_SLOW_BAUD_RATE)

/***************************************************************************//**
* Version 1.01 firmware and below maximum baud rate number.
*******************************************************************************/
#define CMUCOM4_MEDIUM_BAUD_RATE    115200

/***************************************************************************//**
* Version 1.01 firmware and below maximum baud rate string.
*******************************************************************************/
#define CMUCOM4_MEDIUM_BR_STRING    CMUCOM4_V_TO_S(CMUCOM4_MEDIUM_BAUD_RATE)

/***************************************************************************//**
* Version 1.02 firmware and above maximum baud rate number.
*******************************************************************************/
#define CMUCOM4_FAST_BAUD_RATE      250000

/***************************************************************************//**
* Version 1.02 firmware and above maximum baud rate string.
*******************************************************************************/
#define CMUCOM4_FAST_BR_STRING      CMUCOM4_V_TO_S(CMUCOM4_FAST_BAUD_RATE)

/***************************************************************************//**
* Default firmware startup stop bits number.
*******************************************************************************/
#define CMUCOM4_SLOW_STOP_BITS      0

/***************************************************************************//**
* Default firmware startup stop bits string.
*******************************************************************************/
#define CMUCOM4_SLOW_SB_STRING      CMUCOM4_V_TO_S(CMUCOM4_SLOW_STOP_BITS)

/***************************************************************************//**
* Version 1.01 firmware and below necessary stop bits number.
*******************************************************************************/
#define CMUCOM4_MEDIUM_STOP_BITS    0

/***************************************************************************//**
* Version 1.01 firmware and below necessary stop bits string.
*******************************************************************************/
#define CMUCOM4_MEDIUM_SB_STRING    CMUCOM4_V_TO_S(CMUCOM4_MEDIUM_STOP_BITS)

/***************************************************************************//**
* Version 1.02 firmware and above necessary stop bits number.
*******************************************************************************/
#define CMUCOM4_FAST_STOP_BITS      0

/***************************************************************************//**
* Version 1.02 firmware and above necessary stop bits string.
*******************************************************************************/
#define CMUCOM4_FAST_SB_STRING      CMUCOM4_V_TO_S(CMUCOM4_FAST_STOP_BITS)

/***************************************************************************//**
* Serial CMUcom4::begin() post delay in milliseconds.
*******************************************************************************/
#define CMUCOM4_BEGIN_DELAY         1

/***************************************************************************//**
* Serial CMUcom4::end() post delay in milliseconds.
*******************************************************************************/
#define CMUCOM4_END_DELAY           1

/**@endcond********************************************************************/

/***************************************************************************//**
* This is a convenient macro for specifying the Serial port when initializing a
* CMUcam4 or CMUcom4 object.
*******************************************************************************/
#define CMUCOM4_SERIAL              0

/***************************************************************************//**
* This is a convenient macro for specifying the Serial1 port on an Arduino Mega
* when initializing a CMUcam4 or CMUcom4 object.
*******************************************************************************/
#define CMUCOM4_SERIAL1             1

/***************************************************************************//**
* This is a convenient macro for specifying the Serial2 port on an Arduino Mega
* when initializing a CMUcam4 or CMUcom4 object.
*******************************************************************************/
#define CMUCOM4_SERIAL2             2

/***************************************************************************//**
* This is a convenient macro for specifying the Serial3 port on an Arduino Mega
* when initializing a CMUcam4 or CMUcom4 object.
*******************************************************************************/
#define CMUCOM4_SERIAL3             3

/***************************************************************************//**
* This is a hardware abstraction layer for the %CMUcam4 class. The %CMUcom4
* class targets the Ardunio prototyping platform by default.
*******************************************************************************/
class CMUcom4
{

public:

/***************************************************************************//**
* Initialize the %CMUcom4 object to use the default Serial port.
*******************************************************************************/
CMUcom4();

/***************************************************************************//**
* Initialize the %CMUcom4 object to use the @c port Serial port.
* @param [in] port The port.
* @see CMUCOM4_SERIAL
* @see CMUCOM4_SERIAL1
* @see CMUCOM4_SERIAL2
* @see CMUCOM4_SERIAL3
*******************************************************************************/
CMUcom4(int port);

/***************************************************************************//**
* Arduino Serial.begin() wrapper.
* @param [in] baud In bits per second.
* @see http://arduino.cc/en/Serial/Begin
*******************************************************************************/
void begin(unsigned long baud);

/***************************************************************************//**
* Arduino Serial.end() wrapper.
* @see http://arduino.cc/en/Serial/End
*******************************************************************************/
void end();

/***************************************************************************//**
* Arduino Serial.read() wrapper.
* @return The first byte of incoming serial data.
* @see http://arduino.cc/en/Serial/Read
*******************************************************************************/
int read();

/***************************************************************************//**
* Arduino Serial.write() wrapper.
* @param [in] buffer An array to send as a series of bytes.
* @param [in] size The size of the buffer.
* @return The number of bytes written.
* @see http://arduino.cc/en/Serial/Write
*******************************************************************************/
size_t write(const uint8_t * buffer, size_t size);

/***************************************************************************//**
* Arduino Serial.write() wrapper.
* @param [in] str A string to send as a series of bytes.
* @return The number of bytes written.
* @see http://arduino.cc/en/Serial/Write
*******************************************************************************/
size_t write(const char * str);

/***************************************************************************//**
* Arduino Serial.write() wrapper.
* @param [in] c A character to send as a single byte.
* @return The number of bytes written.
* @see http://arduino.cc/en/Serial/Write
*******************************************************************************/
size_t write(uint8_t c);

/***************************************************************************//**
* Arduino Serial.available() wrapper.
* @return The number of bytes available to be read.
* @see http://arduino.cc/en/Serial/Available
*******************************************************************************/
int available();

/***************************************************************************//**
* Arduino Serial.flush() wrapper.
* @see http://arduino.cc/en/Serial/Flush
*******************************************************************************/
void flush();

/***************************************************************************//**
* Arduino Serial.peek() wrapper.
* @return The first byte of incoming serial data available.
* @see http://arduino.cc/en/Serial/Peek
*******************************************************************************/
int peek();

/***************************************************************************//**
* Arduino delay() wrapper.
* @param [in] ms The number of milliseconds to pause for.
* @see http://arduino.cc/en/Reference/Delay
*******************************************************************************/
void delayMilliseconds(unsigned long ms);

/***************************************************************************//**
* Arduino millis() wrapper.
* @return Number of milliseconds since the program started.
* @see http://arduino.cc/en/Reference/Millis
*******************************************************************************/
unsigned long milliseconds();

private:

/***************************************************************************//**
* Selected serial port storage.
* @see CMUCOM4_SERIAL1
* @see CMUCOM4_SERIAL2
* @see CMUCOM4_SERIAL3
*******************************************************************************/
int _port;

//misllis started on the program run
unsigned long _millis;
};

#endif
