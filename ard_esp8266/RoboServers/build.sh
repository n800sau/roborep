#!/bin/bash

PWD=`pwd`
SKETCH=`basename $PWD`
echo $SKETCH
rm -rf /tmp/esp8266
/opt/arduino-1.6.5/arduino -v --verify --pref build.path=/tmp/esp8266 --pref sketchbook.path=.. --board esp8266com:esp8266:esp01 ${SKETCH}.ino &> build.log
echo $?
