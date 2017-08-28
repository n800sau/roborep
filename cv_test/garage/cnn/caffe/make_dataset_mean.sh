#!/usr/bin/env sh

DATABASE=data/db
DATA=data
TOOLS=$CAFFE_ROOT/build/tools

$TOOLS/compute_image_mean $DATABASE/training_lmdb $DATA/dataset_mean.binaryproto

echo "Done."
