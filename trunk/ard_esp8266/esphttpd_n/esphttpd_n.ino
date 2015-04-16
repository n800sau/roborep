#include <ESP8266WiFi.h>
#include <LimitedPtrList.h>
#include <LimitedList.h>
#include "atexec.h"
#include "dbgserial.h"
#include "TelClient.h"

#define MAX_CMD_LENGTH	 25

const char* ssid = "Slow Internet Connection";
const char* password = "1,tpGfhjkz2";

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(23);

LimitedPtrList<TelClient, 10> clients(false);

void dbg_write(uint8_t c)
{
	if(isgraph(c) || isspace(c)) {
		for(int i=0; i<clients.count(); i++) {
			TelClient *client = clients.get(i);
			if(client->dbgMode) {
				client->dbgBuff += (char)c;
			}
		}
	}
}

callback_write_t cbw = {dbg_write};

DbgSerial dbgSerial(UART1, cbw);

void setup() {

	Serial.begin(115200);
	dbgSerial.begin(115200);
	dbgSerial.setDebugOutput(true);
	delay(10);

	// prepare GPIO5
	pinMode(5, OUTPUT);
	digitalWrite(5, 0);
	
	// Connect to WiFi network
	dbgSerial.println();
	dbgSerial.println();
	dbgSerial.print("Connecting to ");
	dbgSerial.println(ssid);
	
	WiFi.begin(ssid, password);
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		dbgSerial.print(".");
	}
	dbgSerial.println("");
	dbgSerial.println("WiFi connected");
	
	// Start the server
	server.begin();
	dbgSerial.println("Server started");

	// Print the IP address
	dbgSerial.println(WiFi.localIP());
}

void loop()
{
	TelClient *client;
	for(int i=0; i<clients.count(); i++) {
		client = clients.get(i);
		if(client->conn.connected()) {
			int n = client->conn.available();
			if(n > 0) {
				for(int j=0; j<n; j++) {
					client->remains += (char)client->conn.read();
				}
				int eolpos;
				do {
					eolpos = client->remains.indexOf('\r');
					if(eolpos >= 0 && client->remains.charAt(eolpos+1) == '\n') {
						eolpos++;
					}
					dbgSerial.println(client->remains);
					dbgSerial.println(eolpos);
					if(client->remains.startsWith("+++AT")) {
						if(eolpos >= 0) {
							// process as command
							String cmd = client->remains.substring(0, eolpos);
							client->remains = client->remains.substring(eolpos+1);
							cmd.trim();
							if(cmd.length() > 0) {
								AtExec(client).parseCommand(cmd);
							}
						}
					} else {
						String data;
						if(eolpos >= 0) {
							data = client->remains.substring(0, eolpos+1);
							client->remains = client->remains.substring(eolpos+1);
						} else {
							data = client->remains;
							client->remains = String();
						}
						// just send to UART
						Serial.print(data);
					}
				} while(eolpos >= 0);
			}
				if(client->dbgBuff.length() > 0) {
					client->conn.write(client->dbgBuff.c_str());
					client->dbgBuff = String();
				}
		} else {
			clients.remove(i--);
		}
	}
	WiFiClient conn = server.available();
	if(conn) {
		client = new TelClient;
		client->conn = conn;
		client->conn.setTimeout(1);
		client->remains = String();
		clients.push(client);
	}
	delay(1);
}
