platformio run &>hex.log && \
cp .pioenvs/arduino_uno/firmware.hex . && \
./mv2pr.sh
echo $?

