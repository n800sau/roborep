#!/bin/bash

# to dinner table
#AZIM=90

#divan
#AZIM=350

#to wall
#AZIM=210

#to balcon
#AZIM=270


AZIM=200

python turn2azim.py ${AZIM} &> turn2azim.log
echo $?

#./mk_big_img.sh
