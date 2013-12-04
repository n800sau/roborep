#!/bin/sh

PIDFILE=/home/n800s/run/oculus.pid
DAEMON=/home/n800s/work/robotarr-code/lib/OculusD/oculusd
DAEMON_ARGS="-f /home/n800s/work/robotarr-code/lib/OculusD/data.txt"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
