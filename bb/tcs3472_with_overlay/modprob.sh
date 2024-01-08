modprobe tcs3472
# from https://github.com/frank-zago/ch341-i2c-spi-gpio/blob/master/README.rst
echo "tcs3472 0x29" > /sys/bus/i2c/devices/i2c-2/new_device
