#!/bin/sh

RUN_AS=n800s
PIDFILE=~/run/bmp085.pid

start-stop-daemon -v --stop --user ${RUN_AS} --pidfile ${PIDFILE} --signal 15 --retry 10
