gpio-admin export 4
cat /sys/devices/virtual/gpio/gpio4/value && \
echo out > /sys/devices/virtual/gpio/gpio4/direction && \
echo 1 > /sys/devices/virtual/gpio/gpio4/value && \
sleep 0.1 && \
echo 0 > /sys/devices/virtual/gpio/gpio4/value && \
sleep 1 && \
echo 1 > /sys/devices/virtual/gpio/gpio4/value && \
gpio-admin unexport 4
