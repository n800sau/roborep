#ino upload
curl -u admin:sekret1 --connect-timeout 5 -F datafile=@`find .build -name firmware.hex` -F baud=115200 http://192.168.1.97/program.cgi
