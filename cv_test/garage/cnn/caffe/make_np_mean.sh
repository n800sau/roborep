export GLOG_minloglevel=3

python -u make_np_mean.py --mean data/dataset_mean.binaryproto &> make_np_mean.log
echo $?
