if ( [ -e /sys/devices/virtual/gpio/gpio18/direction ] && [ -e /sys/devices/virtual/gpio/gpio23/direction ] )
then
    echo busy
else
    echo available
fi
