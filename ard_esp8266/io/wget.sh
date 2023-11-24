#wget http://192.168.1.87/io1/digital/0 -O index.html
#wget http://192.168.1.87/ -O index.html
wget http://192.168.1.87/mode/5/o
wget http://192.168.1.87/mode/6/o
wget -q -O - http://192.168.1.87/digital/6/1 >> wget.log
wget -q -O - http://192.168.1.87/digital/5/1 >> wget.log
wget -q -O - http://192.168.1.87/digital/6/0 >> wget.log
wget -q -O - http://192.168.1.87/digital/6/1 >> wget.log
wget -q -O - http://192.168.1.87/digital/5/0 >> wget.log


