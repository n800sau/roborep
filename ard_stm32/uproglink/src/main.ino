#include <USBComposite.h>
#include <Ticker.h>

USBMultiSerial<2> ms;

int pin_status = LOW;

void display_status()
{
	Serial.print("Tick ");
	Serial.println(pin_status);
	digitalWrite(LED_BUILTIN, pin_status);
	pin_status = !pin_status;
}

Ticker status_timer(display_status, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial.println("Start");
	pinMode(LED_BUILTIN, OUTPUT);
	display_status();
	USBComposite.setProductString("UProgLink");
	USBComposite.setManufacturerString("n800s");
	ms.begin();
//	while (!USBComposite);
	Serial.println("Ready");
	status_timer.start();
}

void loop()
{
	while(ms.ports[0].available()) {
		Serial.write(ms.ports[0].read());
	} 
	while(Serial.available()) {
		ms.ports[0].write(Serial.read());
	} 
	while(ms.ports[1].available()) {
		Serial1.write(ms.ports[1].read());
	} 
	while(Serial1.available()) {
		ms.ports[1].write(Serial1.read());
	} 
	status_timer.update();
}
