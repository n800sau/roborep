rostopic echo -n1 /fchassis/state | grep -E 'heading'
rostopic echo -n1 /fchassis/sonar | grep -E '^range'
