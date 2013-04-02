#!/bin/sh

PIDFILE=/home/n800s/run/hd44780.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/HD44780/hd44780d
start-stop-daemon -v --start --user root --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid root -- ${DAEMON_ARGS}
