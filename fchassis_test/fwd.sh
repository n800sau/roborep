#!/bin/bash

DISTANCE=0.80

python fwd.py $DISTANCE 2> fwd.log
echo $?
