#!/bin/bash

cd `dirname $0`
python picamserver.py &> picamserver.log
cd -
