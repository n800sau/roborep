#include "gpio.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <syslog.h>

std::string gpio_path(int pin)
{
	std::ostringstream rs;
	rs << "/sys/class/gpio/gpio" << pin << "/value";
	return rs.str();
}

void digitalWrite(int pin, int value)
{
	std::ofstream ofs;
	ofs.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	std::string path = gpio_path(pin);
	try {
		ofs.open(path.c_str(), std::ifstream::out);
		ofs << value;
	} catch (const std::exception& e) {
		syslog(LOG_ERR, "Error opening %s: %s", path.c_str(), strerror(errno));
		throw;
	}
}

int digitalRead(int pin)
{
	int rs;
	std::ifstream ifs;
	ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	ifs.open(gpio_path(pin).c_str(), std::ifstream::in);
	ifs >> rs;
	return rs;
}

void wait_interrupt(int pin)
{
	struct pollfd pfd;
	pfd.fd = open(gpio_path(pin).c_str(), O_RDONLY|O_NONBLOCK);
	pfd.events = POLLIN;
	poll(&pfd, 1, -1);
	close(pfd.fd);
}
