#!/bin/bash

#catkin build --verbose -j1 --mem-limit 70%
catkin build --verbose -j1 -p1 --mem-limit 70% &>make.log
echo $?
