const int RED_PIN = 12;
const int GREEN_PIN = 5;
const int YELLOW_PIN = 14;

const int IROUT_PIN = 15;

const int IRIN_PIN = 13; //an IR detector/demodulator is connected to GPIO pin 13

var RECV_PIN = 2;
var LED_NETWORK_ERROR_PIN = 4;
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
	p = require("Ping");
	p.ping({ address: '192.168.1.50' }, function(err, data) {
		pingValue = data.avg;
	});
}
onInit();
