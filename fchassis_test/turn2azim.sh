#!/bin/bash

AZIM=20

python turn2azim.py ${AZIM} &> turn2azim.log
echo $?
