#!/bin/bash

cd /sys/class/gpio
echo 0 > gpio9/value
sleep 1
echo 1 > gpio9/value
