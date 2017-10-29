#export THEANO_FLAGS='floatX=float32,openmp=true'
#export THEANO_FLAGS='device=cpu,blas.ldflags=-lblas -lgfortran'
export THEANO_FLAGS='device=gpu0, floatX=float32'
export CUDA_VISIBLE_DEVICES='0000:01:00.0'
python -u train.py &>train.log
echo $?
