export GLOG_minloglevel=3


python -u predict.py --arch data/garage_deploy.prototxt --classlist data/class.lst \
    --snapshot data/snapshots/garage_iter_5000.caffemodel.h5 \
    --mean data/dataset_mean.binaryproto --test-images ../test_images &> predict.log
echo $?
