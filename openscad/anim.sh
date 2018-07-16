#!/bin/bash

SCADFILE=bb_anycube_lid.scad
#SCADFILE=bb_anycube_case.scad
#SCADFILE=wbbone_box.scad

STEPS=180
PERFRAME=5
FPS=10
PATTERN=%04d.png
DISTANCE=100
Z=150

VNAME=animation.mp4

export DISPLAY=:5

echo >anim.log

for a in $(seq 0 $STEPS)
do
	b=`calc "$a * $PERFRAME / 180 * pi()"`
	x=$(echo `calc "int(sin($b) * $DISTANCE)"`)
	y=$(echo `calc "int(cos($b) * $DISTANCE)"`)
	FILE=$(printf "$PATTERN" $a)
	echo "$x:$y:$FILE" &>> anim.log
	openscad \
			--imgsize=320,240 \
			--autocenter --preview \
			--camera=$x,$y,$Z,0,0,0 \
			-o "$FILE" "$SCADFILE" &>> anim.log
	if [ "$?" != "0" ]
	then
		exit 1
	fi
done && \
ffmpeg -y -i "$PATTERN" -c:v libx264 -r $FPS -pix_fmt yuv420p "${VNAME}" &>ffmpeg.log && \
cd ~/public_html/v && \
./to_webm.sh && \
cd - && \
rm *.png && \
rm "${VNAME}"
echo $?
