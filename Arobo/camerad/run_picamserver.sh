#!/bin/bash

cd `dirname $0`
python picamserver.py 2> picamserver.log
cd -
