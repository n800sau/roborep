cp firmware.hex ../skv500programmer
cd ../skv500programmer && \
./sender_top.sh
echo $?
