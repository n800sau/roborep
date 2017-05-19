#caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_solver.prototxt &> train.log
caffe-segnet/build/tools/caffe train -gpu 0 -solver segnet_solver.prototxt --snapshot snapshots_iter_5677.solverstate &> train.log
echo $?
