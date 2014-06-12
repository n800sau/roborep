#include "pc2nrf.h"
#include <compatibility.h>
#include "../../ino/include/common.h"

#include <syslog.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <hiredis_ext.h>

PC2NRF::PC2NRF():
	ReServant("pc2nrf"), pcounter(0), radio(115, 117), network(radio)
{
}

bool PC2NRF::create_servant()
{
	bool rs = ReServant::create_servant();
	radio.begin();
	network.begin(CHANNEL, PC2NRF_NODE);
	printf("\n");
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

	// check network
	if ( network.available() ) {
		RF24NetworkHeader header;
		payload_t reply;
		network.read(header,&reply,sizeof(reply));
		timespec ts;
		if(clock_gettime(CLOCK_REALTIME_COARSE, &ts) < 0) {
			syslog(LOG_ERR, "Error getting time of day %s", strerror(errno));
		}

		printf("%c", REPLY_MARKER);
		printf(DATA_SEPARATOR "type" DATA_SEPARATOR);
		printf("%u", reply.pload_type);
		printf(DATA_SEPARATOR "secs" DATA_SEPARATOR);
		printf("%ld", ts.tv_sec);
		printf(DATA_SEPARATOR "moffset" DATA_SEPARATOR);
		printf("%ld", ts.tv_nsec / 1000);
		printf(DATA_SEPARATOR "#" DATA_SEPARATOR);
		printf("%lu", reply.counter);
		printf(DATA_SEPARATOR "ms" DATA_SEPARATOR);
		printf("%lu", reply.ms);
		switch(reply.pload_type) {
			case PL_MPU:
				printf(DATA_SEPARATOR "quat_0" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.quaternion[0]);
				printf(DATA_SEPARATOR "quat_1" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.quaternion[1]);
				printf(DATA_SEPARATOR "quat_2" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.quaternion[2]);
				printf(DATA_SEPARATOR "quat_3" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.quaternion[3]);
				printf(DATA_SEPARATOR "grav_x" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.gravity[0]);
				printf(DATA_SEPARATOR "grav_y" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.gravity[1]);
				printf(DATA_SEPARATOR "grav_z" DATA_SEPARATOR);
				printf("%d", reply.d.mpu.gravity[2]);
				break;
			case PL_ACC: 
				printf(DATA_SEPARATOR "acc_x" DATA_SEPARATOR);
				printf("%g", reply.d.acc.raw[0] * (int)reply.d.acc.uScale);
				printf(DATA_SEPARATOR "acc_y" DATA_SEPARATOR);
				printf("%g", reply.d.acc.raw[1] * (int)reply.d.acc.uScale);
				printf(DATA_SEPARATOR "acc_z" DATA_SEPARATOR);
				printf("%g", reply.d.acc.raw[2] * (int)reply.d.acc.uScale);
				break;
		}
		printf(DATA_SEPARATOR "v" DATA_SEPARATOR);
		printf("%u\n", reply.voltage);
/*	} else {
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
*/	}
	ReServant::loop();
}
