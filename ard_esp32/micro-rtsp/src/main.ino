#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <math.h>

#include "config.h"

const char* ssid	 = WIFI_SSID;
const char* password = WIFI_PASSWORD;

#define HOSTNAME "esp32cam"
#define MAX_CLIENT_COUNT 5

OV2640 cam;
WebServer server(80);

unsigned long t=millis();
int counter = 0;
int active_clients = 0;

WiFiClient stream_clients[MAX_CLIENT_COUNT];

// Returns -1 if no free slots
int first_free_slot()
{
	int rs = -1;
	for(int i=0 ; i<MAX_CLIENT_COUNT ; ++i) {
		if(!stream_clients[i] || !stream_clients[i].connected()) {
			rs = i;
			break;
		}
	}
	return rs;
}

void send_frame(WiFiClient &client) {
	server.sendContent(
		"--frame\r\n"
		"Content-Type: image/jpeg\r\n\r\n"
	);
	client.write((char *)cam.getfb(), cam.getSize());
	server.sendContent("\r\n");
}

void handle_jpg_stream(void)
{
	WiFiClient client = server.client();
	int index = first_free_slot();
	if(index < 0) {
		server.send(429, "text/plain", "Limit Exceeded");
	} else {
		stream_clients[index] = client;
		String response = "HTTP/1.1 200 OK\r\n";
		response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
		server.sendContent(response);
	}
}

void handle_jpg(void)
{
	WiFiClient client = server.client();

	if (!client.connected())
	{
		return;
	}
	String response = "HTTP/1.1 200 OK\r\n";
	response += "Content-disposition: inline; filename=capture.jpg\r\n";
	response += "Content-type: image/jpeg\r\n\r\n";
	server.sendContent(response);
	client.write((const char*)cam.getfb(), cam.getSize());
}

void handleNotFound()
{
	String message = "Server is running!\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	server.send(200, "text/plain", message);
}

void setup()
{
	Serial.begin(115200);

	while (!Serial)
	{
		yield();
	}
	Serial.setDebugOutput(true);
	cam.init(esp32cam_config);

	// We start by connecting to a WiFi network

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
	Serial.println("WiFi connected.");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());

	server.on("/jpg_stream", HTTP_GET, handle_jpg_stream);
	server.on("/jpg", HTTP_GET, handle_jpg);
	server.onNotFound(handleNotFound);
	server.begin();

}

void loop()
{
	cam.run();
	server.handleClient();
	active_clients = 0;
	for(int i=0; i<MAX_CLIENT_COUNT; i++) {
		if(stream_clients[i].connected()) {
			active_clients++;
			send_frame(stream_clients[i]);
		}
	}
	unsigned long nt = millis();
	counter++;
	if(nt-t > 1000) {
		Serial.print("clients:");
		Serial.println(active_clients);
		Serial.print("fps:");
		Serial.println(floor(counter*1000/(nt-t)));
		counter = 0;
		t = nt;
	}
}
