#!/bin/sh

PIDFILE=/home/n800s/run/roscore.pid
DAEMON=`which roscore`
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
