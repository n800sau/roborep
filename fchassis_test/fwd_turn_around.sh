#!/bin/bash

DISTANCE=0.50

python fwd_turn_around.py $DISTANCE 2> fwd_turn_around.log
echo $?
