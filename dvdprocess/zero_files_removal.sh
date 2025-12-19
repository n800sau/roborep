#!/bin/bash

# be carefull. this thing renames files

find ./out/ -type f -size 0c -print0 | while read -d $'\0' fname
do
	fsz=`stat -c%s "$fname"`
	if [ $fsz -eq 0 ]
	then
		echo removing "$fname" $fsz ...
		rm "$fname"
	fi
done

