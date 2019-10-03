#!/bin/bash

catkin_make install &>make.log
echo $?
