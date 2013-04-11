#!/bin/sh

for d in `cat dir.list`
do
	if [ -f $d/start.sh ]
	then
		echo "Starting $d"
		$d/start.sh
	fi
done
