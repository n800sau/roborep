#caffe-segnet/build/tools/caffe test -gpu 0 -model segnet_train.prototxt -weights test_weights.caffemodel -iterations 10 &> test.log
caffe-segnet/build/tools/caffe test -gpu 0 -model segnet_train.prototxt -weights test_weights.caffemodel -iterations 10 &> test.log
echo $?
