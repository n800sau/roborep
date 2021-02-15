#sudo ./usb_reset /dev/bus/usb/<bus>/<dev>
#for b in `find /dev/bus/usb/*/*`
#do
#	echo $b
#	sudo ./usb_reset $b
#done
sudo ./usb_reset /dev/bus/usb/008/006
