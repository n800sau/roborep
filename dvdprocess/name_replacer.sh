#!/bin/bash

# be carefull. this thing renames files

find ./out/ -type f -print0 | while read -d $'\0' fname
do
	fsz=`stat -c%s "$fname"`
	if [ $fsz -ne 0 ]
	then
		bfname=`echo "$fname"|sed 's|^./out/|./out.bak/|g'`
		bdname=`dirname "$bfname"`
		mkdir -p "$bdname" && mv "$fname" "$bfname" && touch "$fname"
		echo "$fname" $fsz "->" "$bfname"
	fi
done

