const int pins[] = {
	2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
	A0, A1, A2, A3, A4, A5, A6, A7
};
const int pin_count = sizeof(pins)/sizeof(*pins);

void setup()
{
	Serial.begin(115200);
}

int pin_pos = 0;

void loop()
{
	int pin = pins[pin_pos];
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	Serial.print(pin);
	Serial.println(" LOW");
	delay(500);
	digitalWrite(pin, HIGH);
	Serial.print(pin);
	Serial.println(" HIGH");
	delay(2000);
	if(Serial.available())  {
		Serial.print("pin:");
		Serial.println(pin);
		delay(10000);
		while(Serial.available())  {
			Serial.read();
		}
	}
	digitalWrite(pin, LOW);
	Serial.print(pin);
	Serial.println(" LOW");
	delay(500);
	Serial.println("...");
	pinMode(pin, INPUT);
	pin_pos++;
	if(pin_pos >= pin_count) {
		pin_pos = 0;
		Serial.println("again...");
	}
	delay(500);
}


