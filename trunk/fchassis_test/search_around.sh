#!/bin/bash

CLOCKWISE=1

python search_around.py ${CLOCKWISE} &> search_around.log
echo $?
