/* 
 * File:   spi.h
 * Author: Purinda Gunasekara <purinda@gmail.com>
 * 
 * Created on 24 June 2012, 11:00 AM
 */

#ifndef SPI_H
#define	SPI_H

#include <string>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

using namespace std;

class SPI {
public:
	
	SPI(const char *dev="/dev/spidev1.0");
	uint8_t transfer(uint8_t tx_);
	virtual ~SPI();

private:

	// Default SPI device
	string device;
	// SPI Mode set 
	uint8_t mode;
	// word size
	uint8_t bits;
	// Set SPI speed
	uint32_t speed;
	int fd;

	void init();	
};

#endif	/* SPI_H */

