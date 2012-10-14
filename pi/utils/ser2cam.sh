gpio-admin export 23
gpio-admin export 18
cat /sys/devices/virtual/gpio/gpio18/value && \
cat /sys/devices/virtual/gpio/gpio23/value && \
echo out > /sys/devices/virtual/gpio/gpio18/direction && \
echo out > /sys/devices/virtual/gpio/gpio23/direction && \
echo 0 > /sys/devices/virtual/gpio/gpio18/value
echo 1 > /sys/devices/virtual/gpio/gpio23/value
