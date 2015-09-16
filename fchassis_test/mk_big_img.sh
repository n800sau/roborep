#W=648
#H=486

#W=324
#H=243

W=2592
H=1944

raspistill -w $W -h $H -n --metering average -br 70 -co 60 -o ~/work/picamera/big.jpg
raspistill -w 640 -h 480 -n --metering average -br 70 -co 60 -o ~/public_html/big.jpg
