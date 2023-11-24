rostopic echo -n1 /ow/state | grep -E 'heading'
rostopic echo -n1 /ow/sonar | grep -E '^range'
