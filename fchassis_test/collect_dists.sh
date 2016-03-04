#!/bin/bash

python collect_dists.py &> collect_dists.log
RS=$?
if [ $RS == "0" ]
then
	MP4Box -add v.h264:fps=5 -new ~/public_html/v/v.mp4 &>> collect_dists.log
#	avconv -y -vcodec h264 -i v.mp4 -vcodec copy -acodec copy -r 1 ~/public_html/v/v.mp4 &>> collect_dists.log
fi
echo $RS

