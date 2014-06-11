sudo su -c '
for p in 115 117
do
echo $p >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio$p/direction
chmod g+w /sys/class/gpio/gpio$p/value
chgrp n800s /sys/class/gpio/gpio$p/value
done
'
