#!/bin/sh

PIDFILE=/home/n800s/run/mjpeg_server.pid
DAEMON="`which rosrun` mjpeg_server mjpeg_server _port:=1935"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
