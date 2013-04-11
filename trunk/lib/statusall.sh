#!/bin/sh

for d in `cat dir.list`
do
	if [ -f $d/status.sh ]
	then
		echo "Status of $d:"
		$d/status.sh
	fi
done
