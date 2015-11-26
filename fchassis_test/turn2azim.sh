#!/bin/bash

#divan
#AZIM=350

# to dinner table
#AZIM=90

#to wall
#AZIM=210

#to balcon
#AZIM=270


#AZIM=232
AZIM=210

python turn2azim.py ${AZIM} &> turn2azim.log
#python turn2azim.py ${AZIM}
echo $?

#./mk_big_img.sh
