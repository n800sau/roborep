#!/bin/bash

#divan
#AZIM=350 / 353

# to dinner table
#AZIM=90 / 44

#to wall
#AZIM=210

#to balcon
#AZIM=270


#AZIM=232
AZIM=44

python turn2azim.py ${AZIM} &> turn2azim.log
echo $?

