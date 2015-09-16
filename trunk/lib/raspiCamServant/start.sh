#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
mkdir `eval echo "~${RUN_AS}/run"` 2>/dev/null
PIDFILE=`eval echo "~${RUN_AS}/run/raspiCamServant.pid"`
DAEMON=`pwd`/raspiCamServantd
DAEMON_ARGS="`pwd`/camera.yml"

start-stop-daemon -v --start --user ${RUN_AS} --make-pidfile --pidfile ${PIDFILE} --background --no-close --startas ${DAEMON} --chuid ${RUN_AS} -- ${DAEMON_ARGS}
