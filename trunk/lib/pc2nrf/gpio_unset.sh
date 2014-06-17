#!/bin/sh

sudo su -c '
for p in 39 115 117 72 73
do
echo ${p} >/sys/class/gpio/unexport
done
'
