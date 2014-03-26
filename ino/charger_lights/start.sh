#!/bin/sh

ROS_MASTER_URI=http://localhost:11311

PIDFILE=${HOME}/run/charger_lights.pid
DAEMON="`which rosrun` rosserial_python serial_node.py /dev/ttyO1"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
