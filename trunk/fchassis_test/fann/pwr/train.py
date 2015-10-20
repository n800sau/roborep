#!/usr/bin/python

import json, sys
from pyfann import libfann

#print json.dumps(dir(libfann), indent=2)
#sys.exit()

connection_rate = 1
learning_rate = 0.7
num_input = 1
num_hidden = 3
num_output = 1

desired_error = 0.0001
max_iterations = 2000000
iterations_between_reports = 20000

ann = libfann.neural_net()
ann.create_sparse_array(connection_rate, (num_input, num_hidden, num_output))
#FANN_TRAIN_INCREMENTAL, FANN_TRAIN_BATCH, FANN_TRAIN_QUICKPROP
#ann.set_training_algorithm(libfann.TRAIN_QUICKPROP)
ann.set_learning_rate(learning_rate)
ann.set_activation_function_output(libfann.SIGMOID_SYMMETRIC_STEPWISE)

ann.train_on_file("data", max_iterations, iterations_between_reports, desired_error)

ann.save("net")
