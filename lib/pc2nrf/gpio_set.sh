#!/bin/sh

sudo su -c '
GROUP=n800s
for p in 115 117
do
echo $p >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio${p}/direction
chmod g+w /sys/class/gpio/gpio${p}/value
chgrp ${GROUP} /sys/class/gpio/gpio${p}/value
done
for p in 60
do
echo $p >/sys/class/gpio/export
echo in >/sys/class/gpio/gpio${p}/direction
chmod g+r /sys/class/gpio/gpio${p}/value
chgrp ${GROUP} /sys/class/gpio/gpio${p}/value
done
'
