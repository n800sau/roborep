#export THEANO_FLAGS='floatX=float32,openmp=true'
#export THEANO_FLAGS='device=cpu,blas.ldflags=-lblas -lgfortran'
#export OMP_NUM_THREADS=8
#export GOTO_NUM_THREADS=8
#export MKL_NUM_THREADS=8
python predict.py &> predict.log
echo $?