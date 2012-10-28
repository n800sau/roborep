gpio-admin export 23
cat /sys/devices/virtual/gpio/gpio23/value && \
echo out > /sys/devices/virtual/gpio/gpio23/direction && \
echo 1 > /sys/devices/virtual/gpio/gpio23/value && \
sleep 0.1 && \
echo 0 > /sys/devices/virtual/gpio/gpio23/value && \
sleep 2 && \
echo 1 > /sys/devices/virtual/gpio/gpio23/value && \
gpio-admin unexport 23
