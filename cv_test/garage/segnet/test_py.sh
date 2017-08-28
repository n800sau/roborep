#python test_py.py --model segnet_inference.prototxt  --weights test_weights.caffemodel&>test_py.log
#python test_py.py --model segnet_test.prototxt  --weights test_weights.caffemodel&>test_py.log
#python test_py.py --model segnet_small_test.prototxt  --weights test_weights_small.caffemodel&>test_py.log
python test_py.py --model segnet_tiny_test.prototxt  --weights test_weights_tiny.caffemodel&>test_py.log
echo $?
