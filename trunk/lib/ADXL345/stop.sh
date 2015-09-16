#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
PIDFILE=`eval echo "~${RUN_AS}/run/adxl345.pid"`
echo $PIDFILE
DAEMON=`pwd`/adxl345d

start-stop-daemon -v --stop --user ${RUN_AS} --pidfile ${PIDFILE} --signal 15 --retry 10
