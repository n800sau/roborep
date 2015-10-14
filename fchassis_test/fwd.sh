#!/bin/bash

DISTANCE=3.10

python fwd.py $DISTANCE 2> fwd.log
echo $?
