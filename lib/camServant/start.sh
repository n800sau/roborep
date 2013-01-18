#!/bin/sh

PIDFILE=/home/n800s/run/camd.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/CMUcam4/camTr
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
