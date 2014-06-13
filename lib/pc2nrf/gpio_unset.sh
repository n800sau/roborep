#!/bin/sh

sudo su -c '
for p in 60 115 117
do
echo ${p} >/sys/class/gpio/unexport
done
'
