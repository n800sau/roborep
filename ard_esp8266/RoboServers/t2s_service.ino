#include "t2s_service.h"
#include <ESP8266WiFi.h>

//how many clients should be able to telnet to this ESP8266
#define MAX_TELNET_CLIENTS 10
static WiFiServer server(23);
static WiFiClient serverClients[MAX_TELNET_CLIENTS];


void setupT2Sservice()
{
	//start UART and the server
	Serial.begin(115200);
	server.begin();
	server.setNoDelay(true);
}

void handleT2Sservice()
{
	uint8_t i;
	//check if there are any new clients
	if (server.hasClient()){
		for(i = 0; i < MAX_TELNET_CLIENTS; i++){
			//find free/disconnected spot
			if (!serverClients[i] || !serverClients[i].connected()){
				if(serverClients[i]) serverClients[i].stop();
				serverClients[i] = server.available();
				Serial1.print("New client: "); Serial1.print(i);
				continue;
			}
		}
		//no free/disconnected spot so reject
		WiFiClient serverClient = server.available();
		serverClient.stop();
	}

	//check clients for data
	for(i = 0; i < MAX_TELNET_CLIENTS; i++){
		if (serverClients[i] && serverClients[i].connected()){
			if(serverClients[i].available()){
				//get data from the telnet client and push it to the UART
				while(serverClients[i].available()) Serial.write(serverClients[i].read());
			}
		}
	}

	//check UART for data
	if(Serial.available()){
		size_t len = Serial.available();
		uint8_t sbuf[len];
		Serial.readBytes(sbuf, len);
		//push UART data to all connected telnet clients
		for(i = 0; i < MAX_TELNET_CLIENTS; i++){
			if (serverClients[i] && serverClients[i].connected()){
				serverClients[i].write(sbuf, len);
				delay(1);
			}
		}
	}
}

