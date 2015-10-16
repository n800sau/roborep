#!/bin/bash

CLOCKWISE=1

python search_shapes.py ${CLOCKWISE} &> search_shapes.log
echo $?
