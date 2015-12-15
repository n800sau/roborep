#!/bin/bash

DISTANCE=0.30

python back.py $DISTANCE 2> back.log
echo $?
