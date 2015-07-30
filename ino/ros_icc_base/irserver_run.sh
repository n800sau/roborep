#!/bin/bash

cd `dirname $0`
python irserver.py 2> irserver.log
cd -
