#!/bin/bash

DISTANCE=1.10

python fwd.py $DISTANCE 2> fwd.log
echo $?
