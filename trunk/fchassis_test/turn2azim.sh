#!/bin/bash

AZIM=289

python turn2azim.py ${AZIM} &> turn2azim.log
echo $?
