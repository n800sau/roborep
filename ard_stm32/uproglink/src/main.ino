#include <USBComposite.h>
#include <Ticker.h>
#include <SerialCommands.h>

USBMultiSerial<2> ms;

#define CommandStream (ms.ports[0])
#define PassthroughStream (ms.ports[1])
#define PassthroughEndPoint Serial2
#define DebugStream Serial

int pin_status = LOW;

#define ESP_RESET_PIN PA4
#define ESP_IO00_PIN PA5

char command_buffer[32];
SerialCommands serial_commands(&CommandStream, command_buffer, sizeof(command_buffer), "\n", " ");

void display_status()
{
//	Serial.print("Tick ");
//	Serial.println(pin_status);
	digitalWrite(LED_BUILTIN, pin_status);
	pin_status = !pin_status;
}

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	if(strcmp(cmd, "W") == 0) {
		esp_work_mode();
		sender->GetSerial()->println("W mode");
	} else if(strcmp(cmd, "P") == 0) {
		esp_prog_mode();
		sender->GetSerial()->println("P mode");
	} else if(strcmp(cmd, "R") == 0) {
		esp_reset();
		sender->GetSerial()->println("reset");
	}
}

void esp_reset()
{
	digitalWrite(ESP_RESET_PIN, LOW);
	delay(500);
	digitalWrite(ESP_RESET_PIN, HIGH);
	delay(500);
}

void esp_prog_mode()
{
	digitalWrite(ESP_IO00_PIN, LOW);
	delay(200);
//	PassthroughStream.begin(74880);
	PassthroughEndPoint.begin(74880);
	esp_reset();
}

void esp_work_mode()
{
	digitalWrite(ESP_IO00_PIN, HIGH);
	delay(200);
//	PassthroughStream.begin(115200);
	PassthroughEndPoint.begin(115200);
	esp_reset();
}

Ticker status_timer(display_status, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	Serial.println("Start");
	PassthroughEndPoint.begin(115200);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(ESP_RESET_PIN, OUTPUT);
	pinMode(ESP_IO00_PIN, OUTPUT);
	serial_commands.SetDefaultHandler(cmd_unrecognized);
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

int bytes_from_serial = 0, old_bytes_from_serial = -1;
int bytes_from_usb = 0, old_bytes_from_usb = -1;
int old_baud = 0;

void loop()
{
	serial_commands.ReadSerial();
	int baud = multi_serial_get_baud(PassthroughStream.getPort());
	if(baud != old_baud) {
		old_baud = baud;
		Serial.println(baud);
	}
	while(PassthroughStream.available() || PassthroughEndPoint.available())
	{
		if(PassthroughStream.available())
		{
			int b = PassthroughStream.read();
			bytes_from_usb++;
			PassthroughEndPoint.write(b);
		} 
		if(PassthroughEndPoint.available())
		{
			int b = PassthroughEndPoint.read();
			bytes_from_serial++;
			PassthroughStream.write(b);
		}
	}
	if(old_bytes_from_usb != bytes_from_usb)
	{
		old_bytes_from_usb = bytes_from_usb;
		Serial.print("pushed:");
		Serial.println(bytes_from_usb);
	}
	if(old_bytes_from_serial != bytes_from_serial) {
		old_bytes_from_serial = bytes_from_serial;
		Serial.print(" popped:");
		Serial.println(bytes_from_serial);
	}
	status_timer.update();
}
