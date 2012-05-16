#include <WProgram.h>

//For the RPI-1031 - http://www.sparkfun.com/products/10621 

int __tilt_pin1 = -1;
int __tilt_pin2 = -1;

void rpiSetup(tilt_pin1, tilt_pin2){
 pinMode(tilt_pin1, INPUT);
 pinMode(tilt_pin2, INPUT);
}

int getTiltPosition(){
   int s1 = digitalRead(tilt_pin1);
   int s2 = digitalRead(tilt_pin2);
   return (s1 << 1) | s2; //bitwise math to combine the values
}
