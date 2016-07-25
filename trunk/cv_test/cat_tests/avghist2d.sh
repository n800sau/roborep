THEANO_FLAGS='floatX=float32,openmp=true' OMP_NUM_THREADS=4 python avghist2d.py --conf conf/mycat.json &> avghist2d.log
echo $?