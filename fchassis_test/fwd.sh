#!/bin/bash

DISTANCE=0.50

python fwd.py $DISTANCE 2> fwd.log
echo $?
