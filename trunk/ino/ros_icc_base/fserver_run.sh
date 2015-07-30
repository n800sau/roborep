#!/bin/bash

cd `dirname $0`
python fserver.py 2> fserver.log
cd -
