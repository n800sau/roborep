#!/bin/sh

for d in `cat dir.list`
do
	echo "Making in $d"
	cd $d
	make $1
	rt=$?
	cd -
	if [ $rt -ne 0 ]
	then
		break
	fi
done
