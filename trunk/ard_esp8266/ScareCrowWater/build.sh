#!/bin/bash

platformio run &> build.log
echo $?
