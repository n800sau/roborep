#include "pc2nrf.h"
#include <compatibility.h>
#include "../../ino/include/common.h"

#include <syslog.h>
#include <string.h>
#include <unistd.h>
#include <hiredis_ext.h>

PC2NRF::PC2NRF():
	ReServant("pc2nrf"), pcounter(0), radio(115, 117), network(radio)
{
}

bool PC2NRF::create_servant()
{
	bool rs = ReServant::create_servant();
	radio.begin();
	network.begin(CHANNEL, ACC_NODE);
	return rs;
}

bool PC2NRF::fill_json(json_t *js, int list_id)
{
	return true;
}

void PC2NRF::loop()
{
	// Pump the network regularly
	network.update();

	RF24NetworkHeader header(PC2NRF_NODE);
	payload_t reply;
	reply.pload_type = PL_VOLTAGE;
	reply.voltage = 100;
	reply.ms = __millis();
	reply.counter = ++pcounter;
	bool ok = network.write(header,&reply,sizeof(reply));
	if (ok)
		printf("Replied ok.\n");
	else
		printf("Reply failed.\n");
	ReServant::loop();
}
