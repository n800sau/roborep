#include <ESP8266WiFi.h>
#include <LimitedList.h>
#include "atexec.h"

#define MAX_CMD_LENGTH   25

const char* ssid = "Slow Internet Connection";
const char* password = "1,tpGfhjkz2";

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(23);

LimitedList<WiFiClient, 10> clients(false);

void setup() {
  Serial.begin(115200);
  delay(10);

  // prepare GPIO5
  pinMode(5, OUTPUT);
  digitalWrite(5, 0);
  
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());
}

void loop()
{
	WiFiClient client;
	for(int i=0; i<clients.count(); i++) {
		client = clients.get(i);
		if(client.connected()) {
			if(client.available()) {
				// Read line of the request
				String cmd = client.readStringUntil('\r');
				client.flush();
				cmd.trim();
				if(cmd.length() > 0) {
					AtExec(client).parseCommand(cmd);
				}
			}
		} else {
			clients.remove(i--);
		}
	}
	client = server.available();
	if(client) {
		clients.push(client);
	}
	delay(1);
}
