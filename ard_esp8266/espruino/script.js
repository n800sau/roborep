manager = require("esp-wifi-manager.js");

const BTN_PIN = 4;

const RED_PIN = 12;
const GREEN_PIN = 5;
const YELLOW_PIN = 14;

const IROUT_PIN = 15;

const IRIN_PIN = 13; //an IR detector/demodulator is connected to GPIO pin 13

var RECV_PIN = IRIN_PIN;
var LED_NETWORK_ERROR_PIN = RED_PIN;
var pinValue = "?", pingValue = "?";

setInterval(function() {
	pinValue = digitalRead(RECV_PIN);
//	print("read\n");
}, 1000);

function pageRequest(req, res) {
	res.writeHead(200);
	res.end("Hello World:" + pinValue + "<br/>p:" +
		pingValue + "<br/>Finished");
}
function onInit()
{
	s = require("http");
	s.createServer(pageRequest).listen(80);
	p = require("Wifi");
	p.ping('192.168.1.50', function(time) {
//console.log('time=', time);
		pingValue = time.respTime;
	});

	let isOn = false;
	const interval = 500; // 500 milliseconds = 0.5 seconds
	let LED = YELLOW_PIN;
	let BTN = BTN_PIN;

	//use setupPin(PIN) if you want to setup a button for the Access Point management
	manager.setupPin(BTN);
	manager.init(LED, () => {
		//this device is connected to Internet!
		setInterval(() => {
			isOn = !isOn; // Flips the state on or off
			digitalWrite(LED, isOn); // D2 is the blue LED on the ESP8266 boards
		}, interval);
	})

}
onInit();
