#!/bin/sh

PIDFILE=/home/n800s/run/tcpserver.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/tcpserver/tcpserverd
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
