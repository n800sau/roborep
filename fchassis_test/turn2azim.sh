#!/bin/bash

# to dinner table
#AZIM=90

#divan
#AZIM=350

#to wall
#AZIM=210

AZIM=250

python turn2azim.py ${AZIM} &> turn2azim.log
echo $?
