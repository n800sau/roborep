./export.sh
echo 1 > /sys/devices/virtual/gpio/gpio24/value
echo 0 > /sys/devices/virtual/gpio/gpio17/value
./unexport.sh