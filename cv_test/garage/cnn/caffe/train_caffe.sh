#!/bin/bash

# train the model
TOOLS=$CAFFE_ROOT/build/tools

$TOOLS/caffe train --solver=data/garage_solver.prototxt &> train.log
echo $?
