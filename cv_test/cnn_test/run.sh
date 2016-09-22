#export THEANO_FLAGS='floatX=float32,openmp=true'
export THEANO_FLAGS='device=cpu,blas.ldflags=-lblas -lgfortran'
export OMP_NUM_THREADS=8
export GOTO_NUM_THREADS=8
export MKL_NUM_THREADS=8
#python -u classifier_from_little_data_script_1.py &>run.log
python -u detect_1.py &>run.log
#python classifier_from_little_data_script_2.py &>run.log
#python classifier_from_little_data_script_3.py &>run.log
echo $?
