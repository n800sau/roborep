#!/bin/bash

CLOCKWISE=1

python find_dist.py ${CLOCKWISE} &> find_dist.log
echo $?
