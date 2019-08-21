#!/bin/bash

platformio run &> build.log
#platformio run
#arduino-cli compile --fqbn esp8266:esp8266:generic --build-path build -v src/*.ino &>build.log

echo $?
