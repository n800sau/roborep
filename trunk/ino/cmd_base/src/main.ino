#include "common.h"
#include "printf.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(8,10);

// Network uses that radio
RF24Network network(radio);

// How often to send 'hello world to the other unit
const unsigned long interval = 2000; //ms

// When did we last send?
unsigned long last_sent;

// How many have we sent already
unsigned long packets_sent;

void setup(void)
{
  Serial.begin(57600);
  Serial.println("Commander");
 
  SPI.begin();
  radio.begin();
  network.begin(CHANNEL, BASE_NODE);
}

void loop(void)
{
  // Pump the network regularly
  network.update();

  // If it's time to send a message, send it!
  unsigned long now = millis();
  if ( now - last_sent >= interval  )
  {
    last_sent = now;

    payload_t payload = { PL_CMD, millis(), packets_sent++ };
	payload.d.cmd.cmd = CMD_MPU;

    Serial.print("Sending...");
    Serial.print(payload.counter);
    Serial.print(", ms=");
    Serial.print(payload.ms);
    Serial.print(", ");
    RF24NetworkHeader header(STICK_NODE);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");
  }
	delay(100);
}
