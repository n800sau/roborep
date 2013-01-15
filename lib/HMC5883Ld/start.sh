#!/bin/sh

PIDFILE=/home/n800s/run/hmc5883l.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/HMC5883Ld/hmc5883ld
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
