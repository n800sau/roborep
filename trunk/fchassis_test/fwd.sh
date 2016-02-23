#!/bin/bash

# 1.00 - 2m

DISTANCE=0.20

python fwd.py $DISTANCE 2> fwd.log
echo $?
