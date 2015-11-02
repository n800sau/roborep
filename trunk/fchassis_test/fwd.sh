#!/bin/bash

DISTANCE=0.30

python fwd.py $DISTANCE 2> fwd.log
echo $?
