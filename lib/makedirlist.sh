#!/bin/sh

echo -n >dir.list
for d in `find . -type d -exec ls -d {} \;`
do
	echo "Making in $d"
	if [ -f $d/Makefile ]
	then
		echo $d >>dir.list
	fi
done
