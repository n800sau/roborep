#!/bin/sh

PIDFILE=${HOME}/run/nrf2tcp.pid
DAEMON="`which python` `pwd`/nrf2tcp.py"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
