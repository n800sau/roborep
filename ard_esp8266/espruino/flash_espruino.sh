#!/bin/bash

LOGFNAME=flash.log
source vars.sh
echo >$LOGFNAME
~/.platformio/packages/tool-esptoolpy/esptool.py --port $DEV erase_flash >>$LOGFNAME
echo $?

~/.platformio/packages/tool-esptoolpy/esptool.py --port $DEV --baud 460800 write_flash --flash_size=detect -fm dio 0 espruino_2v04_esp8266_combined_512.bin >>$LOGFNAME
echo $?
