ino upload &> upload.log
#curl --connect-timeout 5 -F datafile=@`find .build -name firmware.hex` -F baud=115200 http://192.168.1.96/program.cgi
echo $?
