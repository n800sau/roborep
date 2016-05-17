#include <ESP8266WiFi.h>

void  setup()
{
	WiFi.mode(WIFI_OFF);
//    Serial.begin(74880);
	Serial.begin(115200);
	Serial.println(F("\nsetup"));

}

void  loop ( )
{
	Serial.println(F("loop"));
	delay(1000);
}
