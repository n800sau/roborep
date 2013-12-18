#!/bin/sh

PIDFILE=/home/n800s/run/roscore.pid
DAEMON=`which roscore`
#DAEMON_ARGS="-p 11111 --pid=$PIDFILE"
start-stop-daemon -v --start --user n800s --background --make-pidfile --pidfile $PIDFILE --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
#start-stop-daemon -v --start --user n800s --pidfile $PIDFILE --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
