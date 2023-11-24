DEV=/sys/bus/i2c/devices/i2c-2/2-0029/iio:device1
echo `cat ${DEV}/in_intensity_red_raw` `cat ${DEV}/in_intensity_green_raw` `cat ${DEV}/in_intensity_blue_raw`
