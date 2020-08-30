#define LED_PIN PB12

int8_t pins[] = {
	PA0,
	PA1,
	PA2,
	PA3,
	PA4,
	PA5,
	PA6,
	PA7,
	PA8,
//	PA9,
//	PA10,
	PA11,
	PA12,
	PA13,
	PA14,
	PA15,
	PB0,
	PB1,
	PB2,
	PB3,
	PB4,
	PB5,
	PB6,
	PB7,
	PB8,
	PB9,
	PB10,
	PB11,
	PB12,
	PB13,
	PB14,
	PB15,
//	PC0,
//	PC1,
//	PC2,
//	PC3,
//	PC4,
//	PC5,
//	PC6,
//	PC7,
//	PC8,
//	PC9,
//	PC10,
//	PC11,
//	PC12,
	PC13,
	PC14,
	PC15,
	-1
};

int8_t cur_pin_index = 26;


void setup() {
	Serial.begin(115200);
	pinMode(LED_PIN, OUTPUT);
	Serial.println("Hello");
}

void loop() {
	Serial.println("Tick");
	digitalWrite(LED_PIN, HIGH);
	delay(300);
	digitalWrite(LED_PIN, LOW);
	delay(300);
}

void loop1() {
	Serial.print("Pin index:");
	Serial.println(cur_pin_index);
	pinMode(pins[cur_pin_index], OUTPUT);
	digitalWrite(pins[cur_pin_index], HIGH);
	delay(200);
	digitalWrite(pins[cur_pin_index], LOW);
	delay(200);
	pinMode(pins[cur_pin_index], INPUT);
	cur_pin_index++;
	if(cur_pin_index >26) {
//	if(pins[cur_pin_index] < 0) {
		cur_pin_index = 0;
	}
}
