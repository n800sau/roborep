#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
mkdir ~/run 2>/dev/null
PIDFILE=~/run/bmp085.pid
DAEMON=`pwd`/bmp085d

start-stop-daemon -v --start --user ${RUN_AS} --make-pidfile --pidfile "${PIDFILE}" --background --no-close --startas ${DAEMON} --chuid ${RUN_AS} -- ${DAEMON_ARGS}
