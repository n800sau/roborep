#!/bin/bash

if [ ! -f /sys/class/gpio/gpio8/value ]
then
	echo Init gpio
	./gpio_on.sh
fi

cd /sys/class/gpio
echo 1 > gpio8/value
echo 0 > gpio9/value
echo 1 > gpio9/value
