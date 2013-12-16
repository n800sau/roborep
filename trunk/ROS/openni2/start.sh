#!/bin/sh

PIDFILE=/home/n800s/run/openni2_server.pid
DAEMON="`which rosrun` openni2_camera openni2_camera_node"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
