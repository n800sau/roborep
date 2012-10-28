./export.sh
echo 0 > /sys/devices/virtual/gpio/gpio17/value
echo 0 > /sys/devices/virtual/gpio/gpio18/value
./unexport.sh
