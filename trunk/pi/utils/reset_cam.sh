gpio-admin export 17
cat /sys/devices/virtual/gpio/gpio17/value && \
echo out > /sys/devices/virtual/gpio/gpio17/direction && \
echo 1 > /sys/devices/virtual/gpio/gpio17/value && \
sleep 0.1 && \
echo 0 > /sys/devices/virtual/gpio/gpio17/value && \
sleep 1 && \
echo 1 > /sys/devices/virtual/gpio/gpio17/value && \
gpio-admin unexport 17
