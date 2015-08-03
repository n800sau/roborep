#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
mkdir ~/run 2>/dev/null
PIDFILE=~/run/hmc5883l.pid
DAEMON=`pwd`/hmc5883ld
start-stop-daemon -v --start --user "${RUN_AS}" --make-pidfile --pidfile "${PIDFILE}" --background --no-close --startas "${DAEMON}" --chuid "${RUN_AS}" -- ${DAEMON_ARGS}
