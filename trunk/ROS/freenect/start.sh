#!/bin/sh

PIDFILE=/home/n800s/run/freenect_server.pid
DAEMON="`which rosrun` freenect_launch freenect-xyz.launch"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
