#!/bin/bash

DISTANCE=0.70

python fwd.py $DISTANCE 2> fwd.log
echo $?
