#!/bin/bash

platformio run -e stm8sblue &> build.log
#platformio run -e stm8sblue
echo $?
