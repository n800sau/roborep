gpio-admin export 24
cat /sys/devices/virtual/gpio/gpio24/value && \
echo out > /sys/devices/virtual/gpio/gpio24/direction && \
echo 1 > /sys/devices/virtual/gpio/gpio24/value
