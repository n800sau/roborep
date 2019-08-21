#!/bin/bash

cd SdRoot
HOSTNAME=esp8266sd.local
URL="$HOSTNAME"/rmdir
PREFIX=test2
for sdname in `find -type d`
do
	for fname in `find $sdname -type f`
	do
		bname=`basename $fname`
		dname=${sdname#.}
		dname=${dname%/}
		dname=$PREFIX$dname
		echo $dname
		curl $URL'?dir='"$dname"
	done
done
