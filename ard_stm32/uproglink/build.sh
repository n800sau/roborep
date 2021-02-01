#!/bin/bash

#platformio run -v -e demo_f030f4
#platformio run -e maple
platformio run -e maple &> build.log
echo $?
