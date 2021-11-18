#include <ESP8266WiFi.h>
#include <FS.h>
#include <WiFiClient.h>
#include <NtpClientLib.h>
//#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <ArduinoJson.h>
#include <FSWebServerLib.h>
#include <Hash.h>
#include <LittleFS.h>
#include <Shell.h>
#include <vector>

#define VERSION "0.6b"

char cmd_buf[256];
const int cmd_buf_size =sizeof(cmd_buf);
int cmd_buf_pos = 0;

char reply_buf[1024];
const int reply_buf_size =sizeof(reply_buf);
int reply_buf_pos = 0;

void update_config()
{
}

void led_blink()
{
	analogWrite(LED_BUILTIN, 97);
	delay(30);
	digitalWrite(LED_BUILTIN, HIGH);
}

void  callbackUSERVERSION(AsyncWebServerRequest *request)
{
	String values = VERSION;
	request->send(200, "text/plain", values);
	values = "";
}

int shell_reader(char * data)
{
	int rs = 0;
	if(cmd_buf_pos > 0) {
		*data = cmd_buf[0];
		rs = 1;
		cmd_buf_pos--;
		if(cmd_buf_pos > 0) {
			memcpy(cmd_buf, cmd_buf+1, cmd_buf_pos);
		}
	} 
	return rs;
}

/**
 * Function to write data to serial port
 * Functions to write to physical media should use this prototype:
 * void my_writer_function(char data)
 */
void shell_writer(char data)
{
	if(reply_buf_pos < reply_buf_size) {
		reply_buf[reply_buf_pos++] = data;
	}
}

int command_ioctrl(int argc, char** argv)
{
	enum ioctrl_switches {
		E_PIN_MODE,
		E_DIGITAL_WRITE,
		E_DIGITAL_READ,
	};

	int i;
	int pin=0xFF, mode =0xFF, val=0xFF;
	int op = 0;

	// If only the program name is given show message to the user about program usage
	if(argc == 1) {
		shell_println("SHELL IO CONTROL UTILITY");
		shell_println("This tool can manipulate the IO pins on the arduino boards using text commands");
		shell_println("Type ioctrl -help for more information.");
		shell_println("");
		return SHELL_RET_SUCCESS;
	}

	// Parse command arguments
	for(i = 0; i<argc; i++)
	{
		if(i==0)
			continue;
		if(!strcmp(argv[i], "-help")) {
			shell_println("This tool can manipulate the IO pins on the arduino boards using text commands");
			shell_println("Arguments in square brackets [ ] are mandatory; arguments in curly brackets { } are optional");
			shell_println("");
			shell_println("AVAILABLE SWITCHES:");
			shell_println("  -p [PIN_NUMBER]     -> \"Sets\" the pin (PIN_NUMBER) to perform other operations on the pin");
			shell_println("  -m [INPUT|OUTPUT]   -> \"Configures\" PIN mode as INPUT or OUTPUT");
			shell_println("  -w [LOW|HIGH]       -> \"Writes\" a digital output value LOW or HIGH");
			shell_println("  -r                  -> \"Reads\" the state of a digital input or output");
			shell_println("  -help               -> Shows this help message");
			shell_println("");
			return SHELL_RET_SUCCESS;
		}
		// Asign pin to read / write or configure
		else if(!strcmp(argv[i], (const char *) "-p")) {
			if(i+1 >argc)
				goto argerror;
			pin =strtol(argv[i+1],0,0);
			shell_printf("#ioctrl-selpin:%d\r\n", pin);
		}
		// Configure the pin mode as input or output
		else if(!strcmp(argv[i], (const char *) "-m")) {
			if(i+1 > argc)
				goto argerror;
			op = E_PIN_MODE;
			if(!strcmp(argv[i+1], (const char *) "INPUT"))
				mode = INPUT;
			else if(!strcmp(argv[i+1], (const char *) "OUTPUT")) 
				mode = OUTPUT;
		}
		// Writes a digital value to IO pin
		else if(!strcmp(argv[i], (const char *) "-w")) {
			if(i+1 > argc)
				goto argerror;
			op = E_DIGITAL_WRITE;
			if(!strcmp(argv[i+1], (const char *) "LOW"))
				val = LOW;
			else if(!strcmp(argv[i+1], (const char *) "HIGH"))
				val = HIGH;
		} 
		else if(!strcmp(argv[i], (const char *) "-r")) {
			op = E_DIGITAL_READ;
		}
	}
	// Check valid pin number
	if(pin == 0xFF) {
		shell_print_error(E_SHELL_ERR_VALUE, 0);
		return SHELL_RET_FAILURE;
	}

	//	Perform operations on the IO pins with the provided information
	switch(op) {
	case E_PIN_MODE:
		shell_print("#ioctrl-mode:");
		if(mode == INPUT)
			shell_println("INTPUT");
		else if( mode == OUTPUT )
			shell_println("OUTPUT");
		else {
			shell_print_error(E_SHELL_ERR_VALUE, "Mode");
			return SHELL_RET_FAILURE;
		}
		pinMode(pin, mode);
		break;

	case E_DIGITAL_WRITE:
		shell_print("#ioctrl-write:");
		if(val == HIGH)
			shell_println("HIGH");
		else if(val == LOW)
			shell_println("LOW");
		else {
			shell_print_error(E_SHELL_ERR_VALUE, "Value");
			return SHELL_RET_FAILURE;
		}
		digitalWrite(pin,val);
		break;

	case E_DIGITAL_READ:
		shell_print("#ioctrl-read:");
		if(digitalRead(pin) == HIGH)
			shell_println("HIGH");
		else
			shell_println("LOW");
		break;
	}

	return SHELL_RET_SUCCESS;
	// Exit point for missing arguments in any part of the function
argerror:
	shell_print_error(E_SHELL_ERR_ARGCOUNT,0);
	shell_println("");
	return SHELL_RET_FAILURE;

}

static std::vector<AsyncClient*> clients; // a list to hold all clients

 /* clients events */
static void handleError(void* arg, AsyncClient* client, int8_t error) {
	Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
	Serial.write((uint8_t*)data, len);
	int sz = min(int(len), int(cmd_buf_size-cmd_buf_pos));
	if(sz > 0) {
		memcpy(cmd_buf+cmd_buf_pos, (uint8_t*)data, sz);
		cmd_buf_pos += sz;
	}

	do {
		shell_task();
	} while(cmd_buf_pos > 0);

	if(reply_buf_pos > 0) {
		// reply to client
		if (client->space() > 32 && client->canSend()) {
			client->add(reply_buf, reply_buf_pos);
			client->send();
			reply_buf_pos = 0;
		}
	}
}

static void handleDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
}

static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time) {
	Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}

/* server events */
static void handleNewClient(void* arg, AsyncClient* client) {
	Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());

	// add to list
	clients.push_back(client);
	
	// register events
	client->onData(&handleData, NULL);
	client->onError(&handleError, NULL);
	client->onDisconnect(&handleDisconnect, NULL);
	client->onTimeout(&handleTimeOut, NULL);
}


void setup()
{
	// setup pwm range 0-100
	analogWriteRange(100);
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	Serial.println();
	Serial.println("Shell...");

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); // turn on

	// WiFi is started inside library
	LittleFS.begin(); // Not really needed, checked inside library and started if needed
	ESPHTTPServer.begin(&LittleFS);

	//set callback for user version
	ESPHTTPServer.setUSERVERSION(VERSION);

	update_config();

	Serial.print("* WiFI connected. IP address: ");
	Serial.print(WiFi.localIP());
	Serial.print(", Hostname: ");
	Serial.println(wifi_station_get_hostname());

	AsyncServer* server = new AsyncServer(1023);
	server->onClient(&handleNewClient, server);
	server->begin();

	// Initialize command line interface (CLI)
	// We pass the function pointers to the read and write functions that we implement below
	// We can also pass a char pointer to display a custom start message
	shell_init(shell_reader, shell_writer, 0);

	// Add commands to the shell
	shell_register(command_ioctrl, "ioctrl");

}

void loop()
{
	shell_task();
}
