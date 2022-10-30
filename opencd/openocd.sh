#sudo openocd -f rpi2.cfg -c "transport select swd" -c "adapter speed 500" -f target/stm32f1x.cfg
#sudo openocd -f raspberrypi2.cfg -c "transport select swd" -c "adapter speed 1000" -f target/stm32f1x.cfg
#openocd -d1 -f raspberrypi2-native.cfg -c "transport select swd" -c "adapter speed 1000" -f target/stm32f1x.cfg
openocd -f raspberrypi2-native.cfg
