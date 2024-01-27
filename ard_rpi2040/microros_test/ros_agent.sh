#https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico

source vars4agent.sh
#micro-ros-agent serial -v --dev "/dev/ttyACM1" -b 115200
micro-ros-agent serial -v --dev "$DEV" -b 115200
#docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:iron serial --dev "$DEV" -b 115200
