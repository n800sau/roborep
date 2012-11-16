#!/bin/sh

gcc -I/usr/include/fastcgi -I../../lib/CMUcam4 server.cpp ../../lib/CMUcam4/libcmucam4.a -lfcgi ../../lib/wiringPi/wiringPi/libwiringPi.a -o server
