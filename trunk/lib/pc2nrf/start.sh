#!/bin/sh

PIDFILE=/home/n800s/run/pc2nrf.pid
DAEMON=/home/n800s/work/roborep/lib/pc2nrf/pc2nrfd
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
