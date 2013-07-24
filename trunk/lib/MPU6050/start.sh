#!/bin/sh

PIDFILE=/home/n800s/run/mpu6050.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/MPU6050/mpu6050d
DAEMON_ARGS="-f /home/n800s/work/sourceforge/robotarr-code/lib/MPU6050/data.txt"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
