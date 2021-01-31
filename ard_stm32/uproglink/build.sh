#!/bin/bash

#platformio run &> build.log
#platformio run -v -e demo_f030f4
platformio run -e maple
echo $?
