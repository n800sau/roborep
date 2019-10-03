python test_caffe.py --arch data/garage_deploy.prototxt \
    --snapshot data/snapshots/garage_iter_5000.caffemodel.h5 \
    --mean data/dataset_mean.binaryproto --val data/val.txt --test-images test_images &> test_caffe.log
echo $?
