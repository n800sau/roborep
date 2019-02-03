#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <Servo.h>
#include <esp_arm/ArmServo.h>

const uint8_t PIN_YAW = 4;
const uint8_t PIN_GRIP = 5;
const uint8_t PIN_UPPER = 12;
const uint8_t PIN_LOWER = 13;

#include "config.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

IPAddress server(192, 168, 1, 45); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

class WiFiHardware {

	public:
	WiFiHardware() {};

	void init()
	{
		// do your initialization here. this probably includes TCP server/client setup
		ensure_connected();
	}

	void ensure_connected()
	{
		if(!client.connected()) {
			client.connect(server, 11411);
		}
	}

	// read a byte from the serial port. -1 = failure
	int read()
	{
		ensure_connected();
		// implement this method so that it reads a byte from the TCP connection and returns it
		//	you may return -1 is there is an error; for example if the TCP connection is not open
		return client.read();				 //will return -1 when it will works
	}

	// write data to the connection to ROS
	void write(uint8_t* data, int length)
	{
		ensure_connected();
		// implement this so that it takes the arguments and writes or prints them to the TCP connection
		for(int i=0; i<length; i++)
			client.write(data[i]);
	}

	// returns milliseconds since start of program
	unsigned long time() {
		 return millis(); // easy; did this one for you
	}
};

class ArmServo
{
public:
	ArmServo(const char *name, byte pin, int vmin, int vmax):
		pin_(pin), vmin_(vmin), vmax_(vmax), name_(name),
		subscriber_(name, &ArmServo::set_pos_callback, this),
		service_server_(name, &ArmServo::service_callback, this)
	{}

	void init(ros::NodeHandle_<WiFiHardware>& nh)
	{
		nh.subscribe(subscriber_);
		nh.advertiseService(service_server_);
	}

	void set_pos_callback(const std_msgs::Int16& msg)
	{
		Serial.print(name_);
		Serial.print(": ");
		Serial.println(msg.data);
		if(msg.data < 0) {
			s.detach();
			Serial.print(name_);
			Serial.println(" detached");
		} else {
			if(!s.attached()) {
				s.attach(pin_);
			}
			int v = msg.data >= 0 ? (msg.data <= 100 ? msg.data : 100) : 0;
			v = map(v, 0, 100, vmin_, vmax_);
			Serial.print(name_);
			Serial.print(" writes ");
			Serial.println(v);
			s.write(v);
			Serial.print(name_);
			Serial.print(" reads ");
			Serial.println(s.read());
		}
	}

	void service_callback(const esp_arm::ArmServo::Request& req, esp_arm::ArmServo::Response& res)
	{
		if(req.wvalue < 0) {
			s.detach();
		} else {
			if(!s.attached()) {
				s.attach(pin_);
			}
			int v = req.wvalue >= 0 ? (req.wvalue <= 100 ? req.wvalue : 100) : 0;
			v = map(v, 0, 100, vmin_, vmax_);
			Serial.print(name_);
			Serial.print(" writes ");
			Serial.println(v);
			s.write(v);
			if(req.msecs > 0) {
				delay(req.msecs);
				s.detach();
			}
		}
		Serial.print(name_);
		if(!s.attached()) {
			Serial.print(" detached");
		}
		Serial.print(" reads ");
		Serial.println(s.read());
		res.rvalue = s.read();
	}

private:
	const byte pin_;
	int vmin_, vmax_;
	String name_;
	Servo s;
	ros::Subscriber<std_msgs::Int16, ArmServo> subscriber_;
	ros::ServiceServer<esp_arm::ArmServo::Request, esp_arm::ArmServo::Response, ArmServo> service_server_;
};

ArmServo yaw("yaw", PIN_YAW, 0, 150);
ArmServo grip("grip", PIN_GRIP, 90, 161);
ArmServo upper("upper", PIN_UPPER, 4, 105);
ArmServo lower("lower", PIN_LOWER, 20, 170);

ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi()
{
	WiFi.begin(ssid, password);
	Serial.print("\nConnecting to "); Serial.println(ssid);
	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
	if(i > 20){
		Serial.print("Could not connect to"); Serial.println(ssid);
		while(1) delay(500);
	}
	Serial.print("Ready! Use ");
	Serial.print(WiFi.localIP());
	Serial.println(" to access client");
}

void setup()
{
	Serial.begin(115200);
	setupWiFi();
	delay(2000);
	nh.initNode();
	yaw.init(nh);
	grip.init(nh);
	upper.init(nh);
	lower.init(nh);
}

void loop() {
	nh.spinOnce();
	delay(500);
}
