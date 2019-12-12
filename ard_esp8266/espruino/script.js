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
}
onInit();
