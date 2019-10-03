#include <Servo.h>

const int headEchoPin = 12; // Echo Pin
const int headTrigPin = 11; // Trigger Pin

const int SONAR_INCR = 5;

const int SONAR_PAN_ANGLE_MIN = 35;
const int SONAR_PAN_ANGLE_MAX = 135;
const int SONAR_PAN_CENTER = SONAR_PAN_ANGLE_MIN + (SONAR_PAN_ANGLE_MAX - SONAR_PAN_ANGLE_MIN) / 2;

const int SONAR_TILT_ANGLE_MIN = 80;
const int SONAR_TILT_ANGLE_MAX = 145;
const int SONAR_TILT_CENTER = SONAR_TILT_ANGLE_MIN + (SONAR_TILT_ANGLE_MAX - SONAR_TILT_ANGLE_MIN) / 2;

// up-down
const int headTiltServoPin = 45;
// left-right
const int headPanServoPin = 46;


Servo head_pan_servo;
Servo head_tilt_servo;


void setup()
{
	Serial.begin(115200);
	Serial2.begin(115200);
	head_pan_servo.attach(headPanServoPin);
	head_tilt_servo.attach(headTiltServoPin);
	delay(5000);
}

#define MAX_COUNT 500

bool done = false;
struct {
	int dist;
	int angle;
} sens[MAX_COUNT];
int num;
unsigned long ms;

String val;

bool pan_dir = true;

void head_scan()
{
	ms = millis();
	Serial2.setTimeout(100);
	Serial.println("Start");
	for(int tilt=SONAR_TILT_ANGLE_MIN; tilt<SONAR_TILT_ANGLE_MAX; tilt+=SONAR_INCR) {
		head_tilt_servo.write(tilt);
		head_pan_servo.write((pan_dir) ? SONAR_PAN_ANGLE_MIN : SONAR_PAN_ANGLE_MAX);
		delay(200);
		// start collecting data
		// to start
		Serial2.print("r");
		delay(1000);
		head_pan_servo.write((pan_dir) ? SONAR_PAN_ANGLE_MAX : SONAR_PAN_ANGLE_MIN);
		pan_dir = !pan_dir;
		// to stop
		Serial2.print("s");
		// to retrieve
		Serial2.print("g");
		bool ok = false;
		for(int i=MAX_COUNT; i<MAX_COUNT; i++) {
			val = Serial2.readStringUntil("#");
			Serial.println(val);
			if(val == "") {
				break;
			} else if (val == "e") {
				ok = true;
				break;
			} else {
				val = Serial.readStringUntil("@");
				Serial.println(val);
				sens[i].dist = val.toInt();
				val = Serial.readStringUntil("@");
				Serial.println(val);
				sens[i].angle = val.toInt();
			}
		}
	}
	// to stop sending data
	Serial2.println("s");
	Serial.println("End");
	Serial.println(millis() - ms);
}

void loop()
{
	if(!done) {
		head_scan();
		done = true;
	}
}
