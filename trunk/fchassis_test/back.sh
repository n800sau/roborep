#!/bin/bash

DISTANCE=1.50

python back.py $DISTANCE 2> back.log
echo $?
