#!/bin/sh

PIDFILE=/home/n800s/run/l3g4200d.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/L3G4200D/l3g4200dd
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
