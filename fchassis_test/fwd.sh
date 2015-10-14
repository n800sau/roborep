#!/bin/bash

DISTANCE=2.10

python fwd.py $DISTANCE 2> fwd.log
echo $?
