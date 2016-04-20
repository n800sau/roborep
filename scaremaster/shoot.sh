# 2592
# 2592 * 0.8 = 2073.6
#raspistill --roi 0.3,0,0.6,1 --width 1555 -s -tl 1000 -v -o ~/sshfs/asus/root/rus_hard/scaremaster/still_`date +%Y-%m-%d-%H:%M:%S`.jpg

BASEPATH=~/sshfs/asus/root/rus_hard/scaremaster

mkdir $BASEPATH/`date +%Y-%m-%d`
raspistill --roi 0.3,0,0.8,1 --width 2073  -o $BASEPATH/`date +%Y-%m-%d`/still_`date +%Y-%m-%d-%H:%M:%S`.jpg
