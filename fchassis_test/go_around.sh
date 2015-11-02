#!/bin/bash

CLOCKWISE=1

python go_around.py ${CLOCKWISE} &> go_around.log
echo $?
