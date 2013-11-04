#ifndef __COMMON_H
#define __COMMON_H

enum PLOAD_TYPE : uint8_t
{
	PL_CMD,
	PL_MPU
};

enum CMD_T : uint8_t
{
	CMD_MPU
};

struct cmd_t {
	CMD_T cmd;
};

struct mpu_t {
	int16_t quaternion[4];
};

// Structure of our payload
struct payload_t {
	PLOAD_TYPE pload_type;
	unsigned long ms;
	unsigned long counter;
	union {
		uint8_t buf[32];
		cmd_t cmd;
		mpu_t mpu;
	} d;
};

//
// Topology
//

// Address of base node
#define BASE_NODE 0

// Address of the stick node
#define STICK_NODE 1

#define CHANNEL 90

#endif //__COMMON_H
