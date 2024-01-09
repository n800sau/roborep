source vars.sh
export PATH=~/.platformio/packages/tool-esptoolpy:$PATH
esptool.py --chip esp32 --port $DEV erase_flash && \
esptool.py --chip esp32 --port $DEV --baud 460800 write_flash -z 0x1000 *.bin
