#!/bin/sh

cd ~/work/robotarr-code/ino/ros_icc_base && \
./fserver_kill.sh
cd -
cd ~/work/robotarr-code/Arobo/camerad && \
./picamserver_kill.sh
cd -
cd ~/work/robotarr-code/lib/HMC5883Ld && \
./stop.sh
cd -
cd ~/work/robotarr-code/lib/ADXL345 && \
./stop.sh
cd -
cd ~/work/robotarr-code/lib/BMP085 && \
./stop.sh
cd -
cd ~/work/robotarr-code/lib/L3G4200D && \
./stop.sh
cd -
