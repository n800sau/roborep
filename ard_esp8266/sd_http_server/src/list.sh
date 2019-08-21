#!/bin/bash

URL=http://esp8266sd.local/ls
echo in /
curl $URL'?dir=/'
echo

#echo in /test2
#curl $URL'?dir=/test2'
#echo

#echo in /test2/edit
#curl $URL'?dir=/test2/edit'
#echo

#echo in /hardware/images
#curl $URL'?dir=/hardware/images'
#echo
