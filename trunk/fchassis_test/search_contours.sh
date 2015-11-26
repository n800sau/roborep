#!/bin/bash

CLOCKWISE=1

python search_contours.py ${CLOCKWISE} &> search_contours.log
echo $?
