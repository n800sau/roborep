#caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_solver.prototxt &> train.log
#caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_solver.prototxt --snapshot snapshots_iter_13000.solverstate &> train.log
#caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_small_solver.prototxt &> train_small.log
#caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_small_solver.prototxt --snapshot snapshots_small_iter_12685.solverstate &> train_small.log
#caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_tiny_solver.prototxt &> train_tiny.log
caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_tiny_solver.prototxt --snapshot snapshots_tiny_iter_28128.solverstate &> train_tiny.log
echo $?
