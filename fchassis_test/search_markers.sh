#!/bin/bash

CLOCKWISE=1

python search_markers.py ${CLOCKWISE} &> search_markers.log
echo $?
