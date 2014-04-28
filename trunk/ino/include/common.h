#ifndef __COMMON_H
#define __COMMON_H

//#include 

enum PLOAD_TYPE
{
	PL_SETSERVO = 1,
	PL_SERVO_STATE,
	PL_MPU,
	PL_ACC
};

#define NO_SET 0xff
// 0xff - no set
typedef struct __stickservo_t {
	uint8_t low;
	uint8_t pan;
	uint8_t tilt;
} stickservo_t;

typedef struct __mpu_t {
	int16_t quaternion[4];
	int16_t gravity[3];
} mpu_t;

typedef struct __acc_t {
	int16_t raw[3];
	uint16_t uScale; //m_Scale * 10000
} acc_t;

// Structure of our payload
struct payload_t {
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

#define ADDRESS_MARKER "ADDRESS:"
#define END_MARKER ":END"
#define CMD_MARKER "STICK:"
#define REPLY_MARKER "REPLY:"
#define REPLY_START_MARKER "REPLYSTART"
#define REPLY_END_MARKER "REPLYEND"
#define SERVO_STATE_MARKER "SERVOSTATE"
#define CONTROLLER_STATE_MARKER "CONTROLLERSTATE"
#define MPU_MARKER "MPU"
#define ACCEL_MARKER "ACCEL"

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


#endif //__COMMON_H
