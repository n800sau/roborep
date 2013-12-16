#!/bin/sh

PIDFILE=/home/n800s/run/openni2_launch.pid
DAEMON="`which roslaunch` openni2_launch openni2.launch"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
