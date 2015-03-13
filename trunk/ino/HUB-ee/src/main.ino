#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"
#include "motor_proc.h"
#include "irdist_proc.h"
#include "presence_proc.h"
#include <voltage.h>
#include <JsonParser.h>

// hmc5883l - 0x1e
// adxl345 - 0x53
// bmp085 - 0x77
// l3g4200d - 0x69


int led = 6;
int lon = 0;

void ParseAnObject()
{
    char json[] = "{\"Name\":\"Blanchon\",\"Skills\":[\"C\",\"C++\",\"C#\"],\"Age\":32,\"Online\":true}\x0d\x0a";

    JsonParser<32> parser;

    Serial.print("Parse ");
    Serial.println(json);

    JsonHashTable hashTable = parser.parseHashTable(json);

    if (!hashTable.success())
    {
        Serial.println("JsonParser.parseHashTable() failed");
        return;
    }

    char* name = hashTable.getString("Name");
    Serial.print("name=");
    Serial.println(name);

    JsonArray skills = hashTable.getArray("Skills");
    Serial.println("skills:");
    for (int i = 0; i < skills.getLength(); i++)
    {
        char* value = skills.getString(i);
        Serial.print(i);
        Serial.print(" ");
        Serial.println(value);
    }

    int age = hashTable.getLong("Age");
    Serial.print("age=");
    Serial.println(age);

    bool online = hashTable.getBool("Online");
    Serial.print("online=");
    Serial.println(online);
}


void setup()
{
	pinMode(led, OUTPUT);
	digitalWrite(led, 1);
	Serial.begin(57600);
//	Serial.println("Starting the I2C interface.");
	Serial.println("Reloading...");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	setup_irdist();
	setup_presence();
	setup_motors();
//	ParseAnObject();
}

void printState()
{
	int v = readVccMv();
	Serial.print("JSON:{\"type\":\"sensors\"");
	Serial.print(",\"V\":");
	Serial.print(v);
	Serial.print(",\"head\":");
	Serial.print(headingDegrees);
	Serial.print(",\"acc_x\":");
	Serial.print(accel_scaled.XAxis);
	Serial.print(",\"gyro_x\":");
	Serial.print((int)gyro.g.x);
	Serial.println();

	Serial.print(", \"T\":");
	Serial.print(bmp.readTemperature());

	Serial.print(", \"P\":");
	Serial.print(bmp.readPressure());

	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	Serial.print(", \"Alt\":");
	Serial.print(bmp.readAltitude());

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
	// Sydney 1018.1 hPa
	Serial.print(", \"R.alt\":");
	Serial.print(bmp.readAltitude(101500));
	Serial.print(", \"IRdist\":");
	Serial.print(distance);
	Serial.print(", \"Presence\":");
	Serial.print(MVcount);

	Serial.println("}");
	Serial.println(".");
}

static char jsonBuf[256];
static int inputPos = 0;
static boolean jsonComplete = false;

void serialEvent() {
	while (Serial.available() && !jsonComplete) {
		char inChar = (char)Serial.read();
//		Serial.println((int)inChar);
		// change 0xd to 0xa
		if( inChar == 0xd ) inChar = 0xa;
		// skip double 0xa or the first 0xa
		if( ! (inChar == 0xa && (inputPos <= 0 || jsonBuf[inputPos-1] == 0xa)) ) {
			jsonBuf[inputPos] = inChar;
			jsonBuf[++inputPos] = 0;
			// check for the end of json
			if((inputPos >= sizeof(jsonBuf) - 1) ||
					(inputPos >= 2 &&
					(inputPos < 3 || jsonBuf[inputPos-3] == 0xa) &&
					(jsonBuf[inputPos-2] == '.') &&
					(jsonBuf[inputPos-1] == 0xa))
			) {
				// end of json
				Serial.println("json end");
				// check for empty json
				if(inputPos >= 5) {
					// not empty
					// remove useless tail
					jsonBuf[inputPos-3] = 0;
					jsonComplete = true;
				}
				inputPos = 0;
			}
		}
	}
}

void execute(const char *cmd, JsonHashTable &data)
{
	if(strcmp(cmd, "sensors") == 0) {
		printState();
	}
}

void loop()
{
	digitalWrite(led, lon);
	lon = !lon;
	process_compass();
	process_accel();
	process_gyro();
	process_bmp085();
	process_irdist();
	process_presence();
	process_motors();

	if (jsonComplete) {
		JsonParser<32> parser;
		JsonHashTable data = parser.parseHashTable(jsonBuf);
		if (!data.success())
		{
			Serial.println("JSON parsing failed of");
			Serial.println(jsonBuf);
		} else {
//			Serial.println(jsonBuf);
			char* cmd = data.getString("command");
			Serial.print("command:");
			Serial.println(cmd);
			execute(cmd, data);
		}
		jsonComplete = false;
	}
	delay(10);
}
