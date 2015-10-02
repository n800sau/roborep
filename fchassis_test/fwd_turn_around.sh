#!/bin/bash

DISTANCE=0.90

python fwd_turn_around.py $DISTANCE 2> fwd_turn_around.log
echo $?
