#!/bin/bash

#PWD=`pwd`
#SKETCH=`basename $PWD`
#echo $SKETCH
#tmp=/tmp/esp8266/$SKETCH
#rm -rf $tmp
#/opt/arduino-1.6.6/arduino -v --verify --pref build.path=$tmp --pref sketchbook.path="${PWD}/.." --board esp8266com:esp8266:esp01 ${SKETCH}.ino &> build.log
#/opt/arduino-1.6.6/arduino -v --verify --pref build.path=$tmp --pref sketchbook.path="${PWD}/.." --board esp8266com:esp8266:generic ${SKETCH}.ino &> build.log
#/opt/arduino-1.6.5-r5/arduino -v --verify --pref build.path=$tmp --pref sketchbook.path="${PWD}/.." --board esp8266com:esp8266:generic ${SKETCH}.ino &> build.log

platformio run
echo $?
