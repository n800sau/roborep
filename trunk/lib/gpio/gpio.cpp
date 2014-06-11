#include "gpio.h"
#include <iostream>
#include <fstream>
#include <sstream>

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
	ofs.open(gpio_path(pin).c_str(), std::ifstream::out);
	ofs << value;
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
