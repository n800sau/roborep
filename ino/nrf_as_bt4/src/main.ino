// >> Inspired by http://dmitry.gr/index.php?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery
// Borrowed from http://doc.lijun.li/misc-nrf24-ble.html

#include "SPI.h"

#define PIN_CE  8 // chip enable
#define PIN_CSN 53   // chip select (for SPI)

// The MAC address of BLE advertizer -- just make one up
const byte mac1[6] = {
	0x11,
	0x22,
	0x33,
	0x44,
	0x55,
	0x66
};

const byte mac2[6] = {
	0x12,
	0x23,
	0x34,
	0x45,
	0x56,
	0x67
};

const byte mac3[6] = {
	0xA2,
	0xFF,
	0x01,
	0xF8,
	0x34,
	0x12
};

uint8_t buf[32];
static const uint8_t chRf[] = {2, 26,80};
static const uint8_t chLe[] = {37,38,39};

void btLeCrc(const uint8_t* data, uint8_t len, uint8_t* dst)
{
	// implementing CRC with LFSR
	uint8_t v, t, d;

	while(len--){
		d = *data++;
		for(v = 0; v < 8; v++, d >>= 1){
			t = dst[0] >> 7;
			dst[0] <<= 1;
			if(dst[1] & 0x80) dst[0] |= 1;
			dst[1] <<= 1;
			if(dst[2] & 0x80) dst[1] |= 1;
			dst[2] <<= 1;

			if(t != (d & 1)){
			  dst[2] ^= 0x5B;
			  dst[1] ^= 0x06;
			}
		}
	}
}

uint8_t  swapbits(uint8_t a)
{
	// reverse the bit order in a single byte
	uint8_t v = 0;
	if(a & 0x80) v |= 0x01;
	if(a & 0x40) v |= 0x02;
	if(a & 0x20) v |= 0x04;
	if(a & 0x10) v |= 0x08;
	if(a & 0x08) v |= 0x10;
	if(a & 0x04) v |= 0x20;
	if(a & 0x02) v |= 0x40;
	if(a & 0x01) v |= 0x80;
	return v;
}

void btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff)
{
	// Implementing whitening with LFSR
	uint8_t  m;
	while(len--){
		for(m = 1; m; m <<= 1){
			if(whitenCoeff & 0x80){
				whitenCoeff ^= 0x11;
				(*data) ^= m;
			}
			whitenCoeff <<= 1;
		}
		data++;
	}
}

static inline uint8_t btLeWhitenStart(uint8_t chan)
{
	//the value we actually use is what BT'd use left shifted one...makes our life easier
	return swapbits(chan) | 2;
}

void btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan)
{
	// Assemble the packet to be transmitted
	// Length is of packet, including crc. pre-populate crc in packet with initial crc value!
	uint8_t i, dataLen = len - 3;
	btLeCrc(packet, dataLen, packet + dataLen);
	for(i = 0; i < 3; i++, dataLen++)
		packet[dataLen] = swapbits(packet[dataLen]);
	btLeWhiten(packet, len, btLeWhitenStart(chan));
	for(i = 0; i < len; i++)
		packet[i] = swapbits(packet[i]); // the byte order of the packet should be reversed as well

}

uint8_t spi_byte(uint8_t byte)
{
	// using Arduino's SPI library; clock out one byte
	SPI.transfer(byte);
	return byte;
}

void nrf_cmd(uint8_t cmd, uint8_t data)
{
	// Write to nRF24's register
	digitalWrite(PIN_CSN, LOW);
	spi_byte(cmd);
	spi_byte(data);
	digitalWrite(PIN_CSN, HIGH);
}

void nrf_simplebyte(uint8_t cmd)
{
	// transfer only one byte
	digitalWrite(PIN_CSN, LOW);
	spi_byte(cmd);
	digitalWrite(PIN_CSN, HIGH);
}

void nrf_manybytes(uint8_t* data, uint8_t len)
{
	// transfer several bytes in a row
	digitalWrite(PIN_CSN, LOW);
	do{
		spi_byte(*data++);
	}while(--len);
	digitalWrite(PIN_CSN, HIGH);
}


void setup()
{
	pinMode(PIN_CSN, OUTPUT);
	pinMode(PIN_CE, OUTPUT);
//	pinMode(11, OUTPUT);
//	pinMode(13, OUTPUT);
	digitalWrite(PIN_CSN, HIGH);
	digitalWrite(PIN_CE, LOW);

	Serial.begin(115200);
	Serial.println("Start LE advertizing");
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);

	// Now initialize nRF24L01+, setting general parameters
	nrf_cmd(0x20, 0x12);  //on, no crc, int on RX/TX done
	nrf_cmd(0x21, 0x00);  //no auto-acknowledge
	nrf_cmd(0x22, 0x00);  //no RX
	nrf_cmd(0x23, 0x02);  //4-byte address
	nrf_cmd(0x24, 0x00);  //no auto-retransmit
	nrf_cmd(0x26, 0x06);  //1MBps at 0dBm
	nrf_cmd(0x27, 0x3E);  //clear various flags
	nrf_cmd(0x3C, 0x00);  //no dynamic payloads
	nrf_cmd(0x3D, 0x00);  //no features
	nrf_cmd(0x31, 32);          //always RX 32 bytes
	nrf_cmd(0x22, 0x01);  //RX on pipe 0

	// Set access addresses (TX address in nRF24L01) to BLE advertising 0x8E89BED6
	// Remember that both bit and byte orders are reversed for BLE packet format
	buf[0] = 0x30;
	buf[1] = swapbits(0x8E);
	buf[2] = swapbits(0x89);
	buf[3] = swapbits(0xBE);
	buf[4] = swapbits(0xD6);
	nrf_manybytes(buf, 5);
	buf[0] = 0x2A;    // set RX address in nRF24L01, doesn't matter because RX is ignored in this case
	nrf_manybytes(buf, 5);
}

int make_packet(const byte mac[], const char name[], const byte payload[], int payload_size)
{
		int i, L=0;
		int namelen = strlen(name);

		buf[L++] = 0x42;  //PDU type, given address is random; 0x42 for Android and 0x40 for iPhone
		buf[L++] = 6 + 3 + 2 + namelen + 2 + payload_size; // length of payload

		// mac
		for(i=0; i<6; i++) {
			buf[L++] = mac[i];
		}

// raw data start

		buf[L++] = 2;   //flags (LE-only, general discoverable mode)
		buf[L++] = 0x01; // type
		buf[L++] = 0x06;  // value

		buf[L++] = namelen + 1;   // length of the name, including type byte
		buf[L++] = 0x08; // type Shorten Local Name
		for(i=0; i<namelen; i++) {
			buf[L++] = name[i]; // value
		}

		buf[L++] = payload_size + 1;   // length of custom data, including type byte
		buf[L++] = 0xFF; // type manufacture Specific Data
		for(i=0; i<payload_size; i++) {
			buf[L++] = payload[i]; //value
		}

		buf[1] = L - 2;

		buf[L++] = 0x55;  //CRC start value: 0x555555
		buf[L++] = 0x55;
		buf[L++] = 0x55;
		// write size
		return L;
}

void publish(const byte mac[], const char name[], const byte payload[], int payload_size)
{
	uint8_t ch = 0, i;  // RF channel for frequency hopping

	// Channel hopping
	for (ch=0; ch<sizeof(chRf); ch++)
	{

		int L = make_packet(mac, name, payload, payload_size);

		nrf_cmd(0x25, chRf[ch]);
		nrf_cmd(0x27, 0x6E);  // Clear flags

		btLePacketEncode(buf, L, chLe[ch]);
		nrf_simplebyte(0xE2); //Clear RX Fifo
		nrf_simplebyte(0xE1); //Clear TX Fifo

		digitalWrite(PIN_CSN, LOW);
		spi_byte(0xA0);
		for(i = 0 ; i < L ; i++) spi_byte(buf[i]);
		digitalWrite(PIN_CSN, HIGH);

		nrf_cmd(0x20, 0x12);  // TX on
		digitalWrite(PIN_CE, HIGH); // Enable Chip
		delay(2);
		digitalWrite(PIN_CE, LOW);   // (in preparation of switching to RX quickly)
	}
}


void loop()
{
	publish(mac1, "CATERPIE", (byte*)"\x2d\x01\x11\x23", 4);
	delay(100);    // Broadcasting interval
	publish(mac2, "PIKACHU", (byte*)"\x2d\x01\x07", 3);
	delay(100);    // Broadcasting interval
	publish(mac3, "POKEMON", (byte*)"\x2d\x01\x07\x33\x56\x11\x04", 7);
	delay(100);    // Broadcasting interval
}
