#export THEANO_FLAGS='floatX=float32,openmp=true'
#export THEANO_FLAGS='device=cpu,blas.ldflags=-lblas -lgfortran'
python -u train.py &>train.log
echo $?
