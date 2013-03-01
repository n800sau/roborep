#!/bin/sh

PIDFILE=/home/n800s/run/bmp085.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/BMP085/bmp085d
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
