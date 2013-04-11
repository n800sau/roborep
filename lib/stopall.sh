#!/bin/sh

for d in `cat dir.list`
do
	if [ -f $d/stop.sh ]
	then
		echo "Stopping $d"
		$d/stop.sh
	fi
done
