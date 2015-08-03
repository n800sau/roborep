#!/bin/sh

RUN_AS=n800s
PIDFILE=~/run/adxl345.pid
DAEMON=`pwd`/adxl345d

start-stop-daemon -v --stop --user ${RUN_AS} --pidfile ${PIDFILE} --signal 15 --retry 10
