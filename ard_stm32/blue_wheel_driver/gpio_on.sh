#!/bin/bash

cd /sys/class/gpio
echo 8 > export
sleep 1
echo 0 > gpio8/value
echo out > gpio8/direction
echo 9 > export
sleep 1
echo 1 > gpio9/value
echo out > gpio9/direction
