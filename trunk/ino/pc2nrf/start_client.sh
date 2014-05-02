#!/bin/sh

PIDFILE=${HOME}/run/nrfredis_client.pid
DAEMON="`which python` `pwd`/nrfredis_client.py"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
