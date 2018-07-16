#include "pc2nrf.h"

#include <syslog.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <hiredis_ext.h>
#include "../gpio/event_gpio.h"

#define MAX_QUEUE_SIZE 1000

//#define PIN_INTERRUPT 39
#define PIN_INTERRUPT 42
//#define PIN_INTERRUPT 73
#define PIN_CE 115
#define PIN_CS 117

void PC2NRF_interrupt_callback(unsigned gpio, void *userptr)
{
	PC2NRF* ths = (PC2NRF*)userptr;
	printf("interrupt\n");
	ths->network_process();
}

PC2NRF::PC2NRF():
	ReServant("pc2nrf"), radio(PIN_CE, PIN_CS), network(radio), has_data(false)
{
	pthread_mutex_init(&mtx, NULL);
}

PC2NRF::~PC2NRF()
{
	pthread_mutex_destroy(&mtx);
}

bool PC2NRF::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		set_redis_list_limit(MAX_QUEUE_SIZE);
		radio.begin();
//		network.begin(CHANNEL, ACC_NODE);
		network.begin(CHANNEL, PC2NRF_NODE);
		add_edge_callback(PIN_INTERRUPT, PC2NRF_interrupt_callback, this);
		add_edge_detect(PIN_INTERRUPT, FALLING_EDGE);
	}
	return rs;
}

bool PC2NRF::fill_json(json_t *js, int list_id)
{
	bool rs = has_data;
	if(has_data) {
		has_data = false;
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
				printf("%g", reply.d.acc.raw[0] * (float)reply.d.acc.uScale);
				printf(DATA_SEPARATOR "acc_y" DATA_SEPARATOR);
				printf("%g", reply.d.acc.raw[1] * (float)reply.d.acc.uScale);
				printf(DATA_SEPARATOR "acc_z" DATA_SEPARATOR);
				printf("%g", reply.d.acc.raw[2] * (float)reply.d.acc.uScale);
				break;
		}
		printf(DATA_SEPARATOR "v" DATA_SEPARATOR);
		printf("%u\n", reply.voltage);
		// form redis data
		json_t *sjs = json_object();
		json_object_set_new(sjs, "type", json_integer(reply.pload_type));
		json_object_set_new(sjs, "secs", json_integer(ts.tv_sec));
		json_object_set_new(sjs, "moffset", json_integer(ts.tv_nsec / 1000));
		json_object_set_new(sjs, "counter", json_integer(reply.counter));
		json_object_set_new(sjs, "ms", json_integer(reply.ms));
		switch(reply.pload_type) {
			case PL_MPU:
				json_object_set_new(sjs, "quat_0", json_integer(reply.d.mpu.quaternion[0]));
				json_object_set_new(sjs, "quat_1", json_integer(reply.d.mpu.quaternion[1]));
				json_object_set_new(sjs, "quat_2", json_integer(reply.d.mpu.quaternion[2]));
				json_object_set_new(sjs, "quat_3", json_integer(reply.d.mpu.quaternion[3]));
				json_object_set_new(sjs, "grav_x", json_integer(reply.d.mpu.gravity[0]));
				json_object_set_new(sjs, "grav_y", json_integer(reply.d.mpu.gravity[1]));
				json_object_set_new(sjs, "grav_z", json_integer(reply.d.mpu.gravity[2]));
				break;
			case PL_ACC:
				json_object_set_new(sjs, "acc_x", json_real(reply.d.acc.raw[0] * (float)reply.d.acc.uScale));
				json_object_set_new(sjs, "acc_y", json_real(reply.d.acc.raw[1] * (float)reply.d.acc.uScale));
				json_object_set_new(sjs, "acc_z", json_real(reply.d.acc.raw[2] * (float)reply.d.acc.uScale));
				break;
		}
		json_object_set_new(sjs, "mv", json_integer(reply.voltage));
		json_object_set_new(js, "data", sjs);
	}
	return rs;
}

void PC2NRF::network_process()
{
	pthread_mutex_lock (&mtx);

	// Pump the network regularly
	network.update();

	// check network
	while ( network.available() ) {
		RF24NetworkHeader header;
		network.read(header,&reply,sizeof(reply));
		has_data = true;
		json2redislist();
	}

	pthread_mutex_unlock(&mtx);
}

void PC2NRF::loop()
{
//	network_process();
	ReServant::loop();
}
