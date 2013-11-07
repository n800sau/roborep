#ifndef __COMMON_H
#define __COMMON_H

//#include 

enum PLOAD_TYPE
{
	PL_SETSERVO = 1,
	PL_SERVO_STATE,
	PL_MPU
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

// Structure of our payload
struct payload_t {
	uint8_t pload_type; //PLOAD_TYPE
	uint64_t ms;
	uint64_t counter;
	union {
		uint8_t buf[32 - sizeof(uint8_t) - sizeof(uint64_t) - sizeof(uint64_t)];
		stickservo_t servo;
		mpu_t mpu;
	} d;
};

#define CMD_MARKER "STICK:"
#define REPLY_MARKER "REPLY:"
#define REPLY_START_MARKER "REPLYSTART"
#define REPLY_END_MARKER "REPLYEND"

//
// Topology
//

// Address of base node
#define BASE_NODE 0

// Address of the stick node
#define STICK_NODE 1

#define CHANNEL 90

#endif //__COMMON_H
