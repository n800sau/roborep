#include "common.h"
#include "printf.h"
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>



// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(8,10);

// Network uses that radio
RF24Network network(radio);

void setup(void)
{
  Serial.begin(57600);
  Serial.println("Commander");
 
  SPI.begin();
  radio.begin();
  network.begin(CHANNEL, STICK_NODE);
}

void loop(void)
{
  // Pump the network regularly
  network.update();

  // Is there anything ready for us?
  while ( network.available() )
  {
    // If so, grab it and print it out
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header,&payload,sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
    Serial.println(payload.d.cmd.cmd);
  }
}
