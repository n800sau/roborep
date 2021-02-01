#include <USBComposite.h>
#include <Ticker.h>

USBMultiSerial<2> ms;

#define CommandStream (ms.ports[0])
#define PassthroughStream (ms.ports[1])
#define PassthroughEndPoint Serial2

int pin_status = LOW;

#define ESP_RESET_PIN PA4
#define ESP_IO00_PIN PA5

void display_status()
{
//	Serial.print("Tick ");
//	Serial.println(pin_status);
	digitalWrite(LED_BUILTIN, pin_status);
	pin_status = !pin_status;
}

void esp_reset()
{
	digitalWrite(ESP_RESET_PIN, LOW);
	delay(200);
	digitalWrite(ESP_RESET_PIN, HIGH);
	delay(200);
}

void esp_prog_mode()
{
	digitalWrite(ESP_IO00_PIN, LOW);
	esp_reset();
}

void esp_work_mode()
{
	digitalWrite(ESP_IO00_PIN, HIGH);
	esp_reset();
}

Ticker status_timer(display_status, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	PassthroughEndPoint.begin(74880);
	Serial.println("Start");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(ESP_RESET_PIN, OUTPUT);
	pinMode(ESP_IO00_PIN, OUTPUT);
	esp_work_mode();
	display_status();
	USBComposite.setProductString("UProgLink");
	USBComposite.setManufacturerString("n800s");
	ms.begin();
//	while (!USBComposite);
	Serial.println("Ready");
	status_timer.start();
	esp_prog_mode();
}

void processCommand(int command, Stream *pstream)
{
	if (command == 'W')
	{
		esp_work_mode();
		pstream->println("W mode");
	} else if (command == 'P')
	{
		esp_prog_mode();
		pstream->println("P mode");
	} else if (command == 'R')
	{
		esp_reset();
		pstream->println("Reset");
	}

}

void loop()
{
	while(CommandStream.available()) {
		int command = CommandStream.read();
		Serial.print("Cmd: ");
		Serial.println(command);
		processCommand(command, &CommandStream);
	} 
	while(PassthroughStream.available()) {
		PassthroughEndPoint.write(PassthroughStream.read());
	} 
	while(PassthroughEndPoint.available()) {
		PassthroughStream.write(PassthroughEndPoint.read());
	} 
	status_timer.update();
}
