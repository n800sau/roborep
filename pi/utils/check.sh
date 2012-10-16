if ( [ -e /sys/devices/virtual/gpio/gpio18/direction ] && [ -e /sys/devices/virtual/gpio/gpio23/direction ] )
then
    echo exists
else
    echo not exist
fi
