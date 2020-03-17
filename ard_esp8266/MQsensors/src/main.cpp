#include <MQUnifiedsensor.h>

//Definitions
#define pin A0 //Analog input 0 of your arduino
#define type 2 //MQ2

MQUnifiedsensor MQ2(pin, type);


float H2, LPG, CO, Alcohol, Propane, Benzene;

void setup() {
    Serial.begin(115200);
    MQ2.inicializar();

}

void loop() {
    MQ2.update();

    H2 =  MQ2.readSensor("H2"); // Return H2 concentration
    LPG =  MQ2.readSensor("LPG"); // Return LPG concentration
    CO =  MQ2.readSensor("CO"); // Return CO concentration
    Alcohol =  MQ2.readSensor("Alcohol"); // Return Alcohol concentration
    Propane =  MQ2.readSensor("Propane"); // Return Propane concentration

	Serial.print("H2:");
	Serial.println(H2);
	Serial.print("LPG:");
	Serial.println(LPG);
	Serial.print("CO:");
	Serial.println(CO);
	Serial.print("Alcohol:");
	Serial.println(Alcohol);
	Serial.print("Propane:");
	Serial.println(Propane);

}

