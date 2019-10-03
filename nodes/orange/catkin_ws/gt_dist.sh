rostopic echo -n1 /ot/state | grep -E 'heading'
rostopic echo -n1 /ot/sonar | grep -E '^range'
rostopic echo -n1 /ot/tof | grep -E '^range'
