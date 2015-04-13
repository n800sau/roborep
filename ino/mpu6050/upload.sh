#socat -d -d -d /dev/tnt0 tcp:192.168.1.95:23 &
#sleep 1
#echo '+++ATIORST' >>/dev/tnt1
#echo '+++ATBAUD 57600' >/dev/tnt1
#printf 'help!\n' >/dev/tnt1
#socat -d -d -d /dev/tnt0 tcp:192.168.1.95:23 &
#ino upload
#curl --connect-timeout 5 -F datafile=@`find .build -name firmware.hex` -F baud=57600 http://192.168.1.95/program.cgi
curl --connect-timeout 5 -F datafile=@`find .build -name firmware.hex` -F baud=115200 http://192.168.1.96/program.cgi

#/usr/share/arduino/hardware/tools/avrdude -C /usr/share/arduino/hardware/tools/avrdude.conf -c arduino -D -U flash:w:.build/pro5v328-59a670ed/firmware.hex:i -b 57600 -P /dev/pts/30 -p atmega328p


