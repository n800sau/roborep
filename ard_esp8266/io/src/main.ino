#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <aREST.h>
#include <aREST_UI.h>

// Create aREST instance
aREST_UI rest = aREST_UI();

// WiFi parameters
//const char* ssid = "your_wifi_network_name";
//const char* password = "your_wifi_network_password";

// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Variables to be exposed to the API
int temperature;
int humidity;

// Declare functions to be exposed to the API
int ledControl(String command);

bool rest_started = false;

void setup(void)
{
  // Start Serial
  Serial.begin(115200);


  // Set the title
  rest.title("ESP8266 Rest UI");

  // Create button to control pin 5
  rest.button(5);
  // Init variables and expose them to REST API
  temperature = 24;
  humidity = 40;
  rest.variable("temperature",&temperature);
  rest.variable("humidity",&humidity);

  // Labels
  rest.label("temperature");
  rest.label("humidity");

  // Function to be exposed
  rest.function("led",ledControl);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("esp8266");

  run_wifimanager();

  // Connect to WiFi
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.println("WiFi connected");

  // Print the IP address
//  Serial.println(WiFi.localIP());
}

void run_rest_server()
{
  // Start the server
  server.begin();
  Serial.println("REST server started");
  rest_started = true;
}

void run_wifimanager()
{
    if(rest_started) {
      server.stop();
      rest_started = false;
    }

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    wifiManager.autoConnect("AutoConnectAP");

    //reset settings - for testing
    //wifiManager.resetSettings();

    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    //wifiManager.setTimeout(120);

    //it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration

    //WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.1
    //WiFi.mode(WIFI_STA);

//    if (!wifiManager.startConfigPortal("OnDemandAP")) {
//      Serial.println("failed to connect and hit timeout");
//      delay(3000);
//      //reset and try again, or maybe put it to deep sleep
//      ESP.reset();
//      delay(5000);
//    }

    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");

    run_rest_server();

}

void loop() {

  if(rest_started) {
    // Handle REST calls
    WiFiClient client = server.available();
    if (client) {
      while(!client.available()){
        delay(1);
      }
      rest.handle(client);
    }
  }
}

// Custom function accessible by the API
int ledControl(String command) {

  // Get state from command
  int state = command.toInt();

  digitalWrite(6,state);
  return 1;
}
