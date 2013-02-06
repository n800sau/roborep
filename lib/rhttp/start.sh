#!/bin/sh

PIDFILE=/home/n800s/run/rhttp.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/rhttp/rhttpd
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
