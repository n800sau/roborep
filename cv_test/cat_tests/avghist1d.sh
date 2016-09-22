THEANO_FLAGS='floatX=float32,openmp=true' OMP_NUM_THREADS=4 python avghist1d.py --conf conf/mycat.json &> avghist1d.log
echo $?