# 2592
# 2592 * 0.8 = 2073.6
#raspistill --roi 0.3,0,0.6,1 --width 1555 -s -tl 1000 -v -o ~/sshfs/asus/root/rus_hard/scaremaster/still_`date +%Y-%m-%d-%H:%M:%S`.jpg

BASEPATH=~/sshfs/asus/root/rus_hard/scaremaster
SUBDIR=`date +%Y-%m-%d`
IMG="still_`date +%Y-%m-%d-%H:%M:%S`.jpg"


mkdir $BASEPATH/$SUBDIR
raspistill --roi 0.3,0,0.8,1 --width 2073  -o $BASEPATH/$SUBDIR/${IMG}

echo $IMG > /home/n800s/work/robotarr-code/scaremaster/cron_shot.last
