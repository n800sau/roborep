#!/bin/bash

cd `dirname $0`
python fchassis.py &> fchassis.log
cd -
