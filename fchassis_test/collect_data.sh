#!/bin/bash

CLOCKWISE=1

python collect_data.py ${CLOCKWISE} &> collect_data.log
echo $?
