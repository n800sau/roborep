#!/bin/bash

DISTANCE=0.10

python fwd.py $DISTANCE 2> fwd.log
echo $?
