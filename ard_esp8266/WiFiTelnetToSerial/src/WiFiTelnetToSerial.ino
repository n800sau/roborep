#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

#include "config.h"

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 5

#define HOSTNAME "orangefb"

const char* ssid     = SSID;
const char* password = PASSWORD;

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

#define DEBUG_SERIAL Serial1

#define CMD_BUF_LEN 30
char cmd_buf[CMD_BUF_LEN];
int cmd_sz = 0;

#define MAX_ARGS 12

void setup() {
	WiFi.hostname(HOSTNAME);
	DEBUG_SERIAL.begin(115200);
	WiFi.begin(ssid, password);
	DEBUG_SERIAL.print("\nConnecting to ");
	DEBUG_SERIAL.println(ssid);
	char i = 0;
	DEBUG_SERIAL.print("\n\r \n\rWorking to connect");

	// Wait for connection
	int conn_status;
	do {
		DEBUG_SERIAL.print(".");
		conn_status = WiFi.status();
		delay(500);
	} while(conn_status != WL_CONNECTED);
	DEBUG_SERIAL.println(".");

	//start UART and the server
	Serial.begin(115200);
	server.begin();
	server.setNoDelay(true);

	if(MDNS.begin(HOSTNAME, WiFi.localIP())) {
		DEBUG_SERIAL.println("mDNS responder started");
		MDNS.addService("telnet", "tcp", 23);
	} else {
		DEBUG_SERIAL.println("Error setting up mDNS responder!");
	}

	DEBUG_SERIAL.print("Ready! Use 'telnet ");
	DEBUG_SERIAL.print(WiFi.localIP());
	DEBUG_SERIAL.println(" 23' to connect");
}

#define CR "\r"
#define NL CR "\n"
#define MSG_OK "OK" NL
#define MSG_ERROR "ERROR" NL
#define MSG_INVALID_CMD "UNKNOWN COMMAND" NL

void cmd_flush()
{
	for(int i=0; i<cmd_sz; i++) {
		Serial.write(cmd_buf[i]);
	}
	cmd_sz = 0;
}

typedef struct config_commands {
	const char *command;
	void (*function)(int argc, char *argv[]);
} config_commands_t;


extern const config_commands_t config_commands[];

void do_help(int argc, char* argv[])
{
	const config_commands_t *p = config_commands;
	String reply;
	while(p->command) {
		reply += String(p->command) + NL;
		p++;
	}
	reply += MSG_OK;
	write2clients(reply.c_str(), reply.length());
}

void config_baud(int argc, char* argv[])
{
	int baud = atoi(argv[1]);
	if(baud > 0) {
		Serial.begin(baud);
	}
	write2clients(MSG_OK, strlen(MSG_OK));
}

void show_version(int argc, char* argv[])
{
	String msg = String(ESP.getSdkVersion()) + NL;
	msg += MSG_OK;
	write2clients(msg.c_str(), msg.length());
}

void do_reset(int argc, char* argv[])
{
	ESP.reset();
}

void show_sysinfo(int argc, char* argv[])
{
	String msg = ESP.getResetInfo() + NL;
	msg += "Hostname:" + WiFi.hostname() + NL;
	msg += "IP:" + WiFi.localIP().toString() + NL;
	msg += MSG_OK;
	write2clients(msg.c_str(), msg.length());
}

const config_commands_t config_commands[] = { 
		{ "HELP", &do_help },
		{ "BAUD", &config_baud },
		{ "VER", &show_version },
		{ "RESET", &do_reset },
		{ "SYSINFO", &show_sysinfo },
//		{ "USEC", &config_usec },
//		{ "UPRI", &config_upri },
//		{ "IFCONFIG", &do_ifconfig },
//		{ "MODE", &config_cmd_mode },
//		{ "STA", &config_cmd_sta },
//		{ "AP", &config_cmd_ap },
//		{ "LON", &io_ledon },
//		{ "LOFF", &io_ledoff },
//		{ "LFLASH", &io_ledflash },
//		{ "VOLTS", &io_volts },
		{ NULL, NULL }
	};



char **config_parse_args(char *buf, char *argc) {
	const char delim[] = " \t";
	char *save, *tok;
	char **argv = (char **)malloc(sizeof(char *) * MAX_ARGS);	// note fixed length
	memset(argv, 0, sizeof(char *) * MAX_ARGS);

	*argc = 0;
	for (; *buf == ' ' || *buf == '\t'; ++buf); // absorb leading spaces
	for (tok = strtok_r(buf, delim, &save); tok; tok = strtok_r(NULL, delim, &save)) {
		argv[*argc] = strdup(tok);
		(*argc)++;
		if (*argc == MAX_ARGS) {
			break;
		}
	}

	return argv;
}

void config_parse_args_free(char argc, char *argv[]) {
	char i;
	for (i = 0; i <= argc; ++i) {
		if (argv[i])
			free(argv[i]);
	}
	free(argv);
}

void config_parse(char *buf, int len) {
	char *lbuf = (char *)malloc(len + 1), **argv;
	char i, argc;
	// we need a '\0' end of the string
	memcpy(lbuf, buf, len);
	lbuf[len] = '\0';

	// remove any CR / LF
	for (i = 0; i < len; ++i)
		if (lbuf[i] == '\n' || lbuf[i] == '\r')
			lbuf[i] = '\0';

	DEBUG_SERIAL.print("Execute ");
	DEBUG_SERIAL.println(lbuf);

	// verify the command prefix
	if (strncmp(lbuf, "+++AT", 5) != 0) {
		return;
	}

	// parse out buffer into arguments
	argv = config_parse_args(&lbuf[5], &argc);
	if (argc == 0) {
		write2clients(MSG_OK, strlen(MSG_OK));
	} else {
		argc--;  // to mimic C main() argc argv
		for (i = 0; config_commands[i].command; ++i) {
			if (strncmp(argv[0], config_commands[i].command, strlen(argv[0])) == 0) {
				config_commands[i].function(argc, argv);
				break;
			}
		}
		if (!config_commands[i].command)
			write2clients(MSG_INVALID_CMD, strlen(MSG_INVALID_CMD));
	}
	config_parse_args_free(argc, argv);
	free(lbuf);
}

void execute_cmd()
{
	cmd_buf[cmd_sz] = 0;
	config_parse(cmd_buf, cmd_sz);
	cmd_sz = 0;
}

void write2clients(const char *sbuf, int len)
{
	//push UART data to all connected telnet clients
	for(int i = 0; i < MAX_SRV_CLIENTS; i++){
		if (serverClients[i] && serverClients[i].connected()){
			serverClients[i].write(sbuf, len);
			delay(1);
		}
	}
}

void loop() {
	int i;
	//check if there are any new clients
	if (server.hasClient()) {
		for(i = 0; i < MAX_SRV_CLIENTS; i++){
			//find free/disconnected spot
			if (!serverClients[i] || !serverClients[i].connected()){
				if(serverClients[i]) serverClients[i].stop();
				serverClients[i] = server.available();
				DEBUG_SERIAL.print("New client: ");
				DEBUG_SERIAL.println(i);
				break;
			}
		}
		if(i == MAX_SRV_CLIENTS) {
			//no free/disconnected spot so reject
			WiFiClient serverClient = server.available();
			serverClient.stop();
			DEBUG_SERIAL.println("No free/disconnected spot. Reject");
		}
	}
	//check clients for data
	for(i = 0; i < MAX_SRV_CLIENTS; i++){
		if (serverClients[i] && serverClients[i].connected()){
			if(serverClients[i].available()){
				//get data from the telnet client and push it to the UART
				while(serverClients[i].available()) {
					int b = serverClients[i].read();
					if(b == '+') {
						if(cmd_sz < 3) {
							cmd_buf[cmd_sz++] = b;
							b = -1;
						} else {
							cmd_flush();
						}
					}
					if(cmd_sz >= 3 && b >= 0) {
						cmd_buf[cmd_sz++] = b;
						if(cmd_sz >= CMD_BUF_LEN - 4) {
							cmd_flush();
						} else if(b < ' ') {
							execute_cmd();
						}
						b = -1;
					}
					if(cmd_sz == 0 && b >= 0) {
						Serial.write(b);
					}
				}
			}
		}
	}
	//check UART for data
	if(Serial.available()){
		size_t len = Serial.available();
		char sbuf[len];
		Serial.readBytes(sbuf, len);
		write2clients(sbuf, len);
	}
}
