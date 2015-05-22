#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <aREST.h>
#include <Ticker.h>
#include <PString.h>

//how many clients should be able to telnet to this ESP8266
#define MAX_TELNET_CLIENTS 10
const char* ssid = "Slow Internet Connection";
const char* password = "1,tpGfhjkz2";

WiFiServer server(23);
WiFiClient serverClients[MAX_TELNET_CLIENTS];

ESP8266WebServer wserver (80);

// Create aREST instance
aREST rest;
// Create an instance of the server
WiFiServer rserver(8080);

const int led = 13;
bool serPort = false;

void handleRoot() {
	digitalWrite ( led, 1 );
	char temp[400];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf ( temp, 400,

"<html>\
	<head>\
		<meta http-equiv='refresh' content='5'/>\
		<title>ESP8266 Demo</title>\
		<style>\
			body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
		</style>\
	</head>\
	<body>\
		<h1>Hello from ESP8266!</h1>\
		<p>Uptime: %02d:%02d:%02d</p>\
		<img src=\"/test.svg\" />\
	</body>\
</html>",

		hr, min % 60, sec % 60
	);
	wserver.send ( 200, "text/html", temp );
	digitalWrite ( led, 0 );

}

#define MESSAGE_SIZE 2000
String message;

void setup() {
	pinMode (4, OUTPUT );
	digitalWrite(4, 1);
	Serial1.begin(115200);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
//	WiFi.softAP("ESP Monster 96", "n42n3hofdS");
	Serial1.print("\nConnecting to "); Serial1.println(ssid);
	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
	if(i == 21){
		Serial1.print("Could not connect to"); Serial1.println(ssid);
		while(1) delay(500);
	}
	//start UART and the server
	Serial.begin(115200);
	server.begin();
	server.setNoDelay(true);
	
	Serial1.print("Ready! Use 'telnet ");
	Serial1.print(WiFi.localIP());
	Serial1.println(" 21' to connect");

	wserver.on ( "/", handleRoot );
	wserver.on ( "/test.svg", drawGraph );
	wserver.begin();
	Serial1.println ( "HTTP server started" );

	// Give name and ID to device
	rest.set_id((char*)"1");
	rest.set_name((char*)"esp8266");

	// Function to be exposed
	rest.function((char*)"ioreset", ioReset);
	// Function to change serial
	rest.function((char*)"selser", selSer);

	// Command function
	rest.function((char*)"command", executeRESTCommand);

	message.reserve(MESSAGE_SIZE);
	rest.variable((char*)"message", &message);

	// Start the REST server
	rserver.begin();
	Serial1.println("REST server started");
}

Ticker flipper;

void resetOff()
{
	digitalWrite(4, 1);
}

int ioReset(String param)
{
	digitalWrite(4, 0);
	flipper.once(1, resetOff);
	return 0;
}

int selSer(String param)
{
	bool port = param.toInt();
	if(port != serPort) {
		Serial.swap();
		serPort = port;
	}
	return serPort;
}

void drawGraph() {
	String out = "";
	char temp[100];
	out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
	out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
	out += "<g stroke=\"black\">\n";
	int y = rand() % 130;
	for (int x = 10; x < 390; x+= 10) {
		int y2 = rand() % 130;
		sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
		out += temp;
		y = y2;
	}
	out += "</g>\n</svg>\n";

	wserver.send ( 200, "image/svg+xml", out);
}

void loop() {
	// handle WEB
	wserver.handleClient();

	// handle telnet 2 UART
	handleTCP2Ser();

	// handle REST calls
	handleREST();
}

void handleTCP2Ser()
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

void handleREST()
{
	if (rserver.hasClient()) {
		WiFiClient rclient = rserver.available();
		while(!rclient.available()){
			delay(1);
		}
		rest.handle(rclient);
	}
}

int executeRESTCommand(String command)
{
	int rs = -1;
	static char msgbuf[MESSAGE_SIZE];
	PString pmsg(msgbuf, MESSAGE_SIZE);
	if(command.equalsIgnoreCase("info")) {
		WiFi.printDiag(pmsg);
		message = msgbuf;
		rs = 0;
	}
	return rs;
}
