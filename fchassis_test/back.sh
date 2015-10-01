#!/bin/bash

DISTANCE=0.50

python back.py $DISTANCE 2> back.log
echo $?
