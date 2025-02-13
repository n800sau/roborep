#ifndef __COMMON_H
#define __COMMON_H

#ifdef __GNUC__
#define STRUCT_PACKED __attribute__((__packed__))
#else
#define STRUCT_PACKED
#endif

enum PLOAD_TYPE
{
	PL_SETSERVO = 1,
	PL_SERVO_STATE,
	PL_MPU,
	PL_ACC,
	PL_VOLTAGE
};

#define NO_SET 0xff

// 0xff - no set
typedef struct STRUCT_PACKED __stickservo_t {
	uint8_t low;
	uint8_t pan;
	uint8_t tilt;
} stickservo_t;

typedef struct STRUCT_PACKED __mpu_t {
	int16_t quaternion[4];
	int16_t gravity[3];
} mpu_t;

typedef struct STRUCT_PACKED __acc_t {
	int16_t raw[3];
	uint16_t uScale; //m_Scale * 10000
} acc_t;

// Structure of our payload
struct STRUCT_PACKED payload_t {
	uint8_t pload_type; //PLOAD_TYPE
	uint32_t ms; //TIME
	uint32_t counter;
	uint16_t voltage;
	union {
		uint8_t buf[32 - sizeof(uint8_t) - sizeof(uint32_t) - sizeof(uint32_t) - sizeof(uint16_t)];
		stickservo_t servo;
		mpu_t mpu;
		acc_t acc;
	} d;
};

const char DATA_MARKER = 'D';
const char READY_MARKER = '!';
const char ACK_MARKER = '.';
const char ADDRESS_MARKER = 'A';
const char END_MARKER = 'E';
const char CMD_MARKER = 'P';
const char REPLY_MARKER = 'R';
const char REPLY_START_MARKER = 'B';
const char SERVO_STATE_MARKER = 'M';
const char CONTROLLER_STATE_MARKER = 'S';
const char MPU_MARKER = 'U';
const char ACCEL_MARKER = 'X';

#define DATA_SEPARATOR "\t"

//
// Topology
//

// Address of pc 2 nrf network node
#define PC2NRF_NODE 00

// Address of base node
#define BASE_NODE 01

// Address of the stick node
#define STICK_NODE 02

// Address of the acc node
#define ACC_NODE 03

// Address of the stick node
#define NRF_MPU6050_NODE 04

#define CHANNEL 90

#define SERIAL_DEBUG

#define BEACON_CODE 0xFA441211

#endif //__COMMON_H
