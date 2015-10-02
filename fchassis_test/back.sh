#!/bin/bash

DISTANCE=0.90

python back.py $DISTANCE 2> back.log
echo $?
