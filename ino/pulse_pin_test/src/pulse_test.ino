// seq: DTR 0 .. 400 .. 0 .. 200 .. 1 - boot_mode
//      RTS 0 .. ...... 1 ......... 1 - reset

// A4 - DTR
// A5 - RTS
int pins[] = {A1, A2, A4, A5};
String s_pins[] = {"1", "2", "4", "5"};
//unsigned long duration;
const int pincount = sizeof(pins)/sizeof(pins[0]);

void setup()
{
	for(int i=0; i<pincount; i++) {
		pinMode(pins[i], INPUT);
	}
	Serial.begin(115200);
	Serial.println("hello");
}

String old_state = "";
unsigned long m = 0;

void loop()
{
	String state = "";
	for(int i=0; i<pincount; i++) {
		state += (digitalRead(pins[i]) == HIGH) ? s_pins[i] : "O";
	}
	if(state != old_state) {
		Serial.print(old_state);
		Serial.print(" -> ");
		Serial.print(state);
		unsigned long new_m = millis();
		if(m > 0) {
			Serial.print(" - ");
			Serial.print(new_m - m);
		}
		Serial.println();
		m = new_m;
		old_state = state;
	}
//	duration = pulseIn(pin, HIGH);
//	if(duration > 0) {
//		Serial.println(duration);
//	}
	delay(200);
}
