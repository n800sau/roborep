#!/bin/bash

AZIM=10

python turn2azim.py ${AZIM} &> turn2azim.log
echo $?
