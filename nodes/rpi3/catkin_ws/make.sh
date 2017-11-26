#!/bin/bash

catkin build --verbose -j1 --mem-limit 50% &>make.log && \
echo source install/setup.bash
echo $?
