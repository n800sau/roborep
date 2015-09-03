#!/bin/bash

CLOCKWISE=1

python move_around.py ${CLOCKWISE} &> move_around.log
echo $?
