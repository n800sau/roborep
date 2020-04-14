#!/bin/bash

platformio run -e bluepill --verbose  &> build.log
#platformio run -e demo_f030f4  &> build.log
#platformio run -e stm8sblue --verbose  &> build.log
#platformio run -e esp8266 --verbose  &> build.log
#platformio run -e demo_f030f4 --verbose  &> build.log
#platformio run -e bluepill
echo $?
