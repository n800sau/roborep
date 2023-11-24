#!/bin/bash

platformio run -e maple &> build.log
#platformio run -e maple
echo $?
