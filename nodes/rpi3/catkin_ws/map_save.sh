rosrun map_server map_saver map:=/rpi3/map -f map_`date +%Y%m%d-%H%M%S`.map &>map_save.log
echo $?
