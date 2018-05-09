#!/bin/bash

platformio run -t upload &>upload.log
#platformio run -t upload
echo $?
