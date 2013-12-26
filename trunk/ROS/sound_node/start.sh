#!/bin/sh

PIDFILE=/home/n800s/run/sound_node.pid
DAEMON="`which rosrun` sound_play soundplay_node.py"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
