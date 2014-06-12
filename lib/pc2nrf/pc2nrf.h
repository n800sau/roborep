#ifndef PC2NRF_h
#define PC2NRF_h

#include <reservant.h>
#include <RF24Network.h>
#include <RF24.h>
#include "../../ino/include/common.h"

class PC2NRF:public ReServant
{
	private:

		// nRF24L01(+) radio attached using Getting Started board 
		RF24 radio;

		// Network uses that radio
		RF24Network network;

		// has data mark
		bool has_data;

		// data
		payload_t reply;

	protected:
		virtual bool create_servant();
		virtual void loop();
		virtual bool fill_json(json_t *js, int list_id);

	public:

		PC2NRF();
};

#endif



