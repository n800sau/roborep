#!/bin/bash

DISTANCE=1.30

python fwd.py $DISTANCE 2> fwd.log
echo $?
