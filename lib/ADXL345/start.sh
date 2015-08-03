#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
mkdir ~/run 2>/dev/null
PIDFILE=~/run/adxl345.pid
DAEMON=`pwd`/adxl345d

start-stop-daemon -v --start --user ${RUN_AS} --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid ${RUN_AS} -- ${DAEMON_ARGS}
