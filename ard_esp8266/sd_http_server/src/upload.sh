#!/bin/bash

cd html
HOSTNAME=esp8266sd.local
URL="$HOSTNAME"/up
PREFIX=
for sdname in `find -type d`
do
	for fname in `find $sdname -type f`
	do
		bname=`basename $fname`
		dname=${sdname#.}
		dname=${dname%/}
		dname=$PREFIX$dname
		echo $dname
		curl -F "file=@$fname" $URL'?dir='"$dname"
	done
done
