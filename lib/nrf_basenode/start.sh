#!/bin/sh

PIDFILE=/home/n800s/run/nrf_basenode.pid
DAEMON=/home/n800s/work/robotarr-code/lib/nrf_basenode/nrf_basenoded
DAEMON_ARGS="-f /home/n800s/work/robotarr-code/lib/nrf_basenoded/data.txt"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
