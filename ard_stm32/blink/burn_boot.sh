#python ./stm32loader.py -p /dev/ttyUSB0 -w generic_boot20_pc13.bin
#python ./stm32loader.py -p /dev/ttyUSB0 -w generic_boot20_pb12.bin
#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w generic_boot20_pb12.bin -v -g 0x0 /dev/ttyUSB0
#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w generic_boot20_pc13.bin -v -g 0x0 /dev/ttyUSB0
#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w maple_rev5_boot20.bin -v /dev/ttyUSB0
#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w maple_mini_boot20.bin -v -g 0x0 /dev/ttyUSB0
#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w gd32f1_generic_boot20_pc13.bin -v -g 0x0 /dev/ttyUSB0
#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w Example_Bootloader.bin -v -g 0x0 /dev/ttyUSB0
~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w $1 -v /dev/ttyUSB0

#~/.platformio/packages/tool-stm32duino/stm32flash/stm32flash -w generic_boot20_pc13.bin -v -g 0x0 /dev/ttyACM0
