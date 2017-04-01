export GLOG_minloglevel=3

#SRC_PATH=../test_images
SRC_PATH=~/sshfs/asus/root/rus_hard/garage/2016-11-01

#python -u predict.py --arch data/garage_deploy.prototxt --classlist data/class.lst \
#    --snapshot data/snapshots/garage_iter_55000.caffemodel.h5 \
#    --mean data/dataset_mean.binaryproto --test-images $SRC_PATH --outpath output \
#    --dim 160x160 &> predict.log

python -u predict.py --arch data/garage_deploy.prototxt --classlist data/class.lst \
    --mean data/dataset_mean.binaryproto --test-images $SRC_PATH --outpath output \
    --dim 160x160 &> predict.log
echo $?
