platformio run &>hex.log && \
mv .pioenvs/arduino_uno/firmware.hex . && \
./mv2pr.sh
echo $?


