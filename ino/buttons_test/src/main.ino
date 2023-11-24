#include <keylib.h>

const uint8_t oline[] = {11, 12, 13};
const uint8_t iline[] = {8, 9, 10};

readKey keylib = readKey();

void setup()
{
	Serial.begin(115200);
	keylib.begin(sizeof(oline), sizeof(iline), oline, iline);
	Serial.println("\nButton test");
}

void loop()
{
	byte key = keylib.read_key_debounce();
	Serial.println(key, HEX);
}
