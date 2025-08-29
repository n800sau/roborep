#include <Arduino.h>

#ifndef LED_BUILTIN
		#define LED_BUILTIN PC13
#endif

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
//	Serial1.begin(9600);
//	Serial2.begin(9600);
//	Serial3.begin(9600);
}

// stlink output
void put_char(char c)
{
    asm (
    "mov r0, #0x03\n"   /* SYS_WRITEC */
    "mov r1, %[msg]\n"
    "bkpt #0xAB\n"
    :
    : [msg] "r" (&c)
    : "r0", "r1"
    );
}

void send_command(int command, void *message)
{
   asm("mov r0, %[cmd];"
       "mov r1, %[msg];"
       "bkpt #0xAB"
         :
         : [cmd] "r" (command), [msg] "r" (message)
         : "r0", "r1", "memory");
}


void loop()

{
	digitalWrite(LED_BUILTIN, 1);
//	Serial1.println("Serial LED OFF");
	Serial.println("Serial zero LED OFF");
//	Serial2.println("Serial DUE LED OFF");
//	Serial3.println("Serial TRE LED OFF");

	delay(1000);

	digitalWrite(LED_BUILTIN, 0);
//	Serial1.println("Serial LED ON");
	Serial.println("Serial zero LED ON");
//	Serial2.println("Serial DUE LED ON");
//	Serial3.println("Serial TRE LED ON");

	delay(1000);
// output via stlink
	const char s[] = "Hello world\n";
	uint32_t m[] = { 2/*stderr*/, (uint32_t)s, sizeof(s)/sizeof(*s)-1 };
	send_command(0x05/* some interrupt ID */, m);

}
