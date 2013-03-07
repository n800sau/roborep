#!/bin/sh

PIDFILE=/home/n800s/run/mag3110.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/MAG3110/mag3110d
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
