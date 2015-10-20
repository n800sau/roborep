#!/bin/bash

DISTANCE=1.00

python back.py $DISTANCE 2> back.log
echo $?
