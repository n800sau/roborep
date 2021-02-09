const int stm32_boot_pin = 3;
const int stm32_reset_pin = 2;
const int esp_reset_pin = A0;
const int esp_io00_pin = A1;

// the setup routine runs once when you press reset:
void setup()
{
	Serial.begin(115200);

	// initialize the digital pin as input (work mode)
	pinMode(stm32_boot_pin, OUTPUT);
	pinMode(stm32_reset_pin, OUTPUT);
	digitalWrite(stm32_boot_pin, HIGH);
	digitalWrite(stm32_reset_pin, HIGH);
	pinMode(esp_io00_pin, OUTPUT);
	pinMode(esp_reset_pin, OUTPUT);
	digitalWrite(esp_io00_pin, HIGH);
	digitalWrite(esp_reset_pin, HIGH);
	Serial.println("Ready");
}

void stm32_reset()
{
	digitalWrite(stm32_reset_pin, LOW);
	delay(300);
	digitalWrite(stm32_reset_pin, HIGH);
}

void stm32_prog_mode()
{
	digitalWrite(stm32_boot_pin, HIGH);
	delay(300);
	stm32_reset();
}

void stm32_work_mode()
{
	digitalWrite(stm32_boot_pin, LOW);
	delay(300);
	stm32_reset();
}

void esp_reset()
{
	digitalWrite(esp_reset_pin, LOW);
	delay(300);
	digitalWrite(esp_reset_pin, HIGH);
}

void esp_prog_mode()
{
	digitalWrite(esp_io00_pin, LOW);
	delay(300);
	esp_reset();
}

void esp_work_mode()
{
	digitalWrite(esp_io00_pin, HIGH);
	delay(300);
	esp_reset();
}

void loop()
{
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == 'W') {
			stm32_work_mode();
			Serial.println("W mode");
		} else if (incomingByte == 'P') {
			stm32_prog_mode();
			Serial.println("P mode");
		} else if (incomingByte == 'w') {
			esp_work_mode();
			Serial.println("w mode");
		} else if (incomingByte == 'p') {
			esp_prog_mode();
			Serial.println("p mode");
		} 
	}
}
