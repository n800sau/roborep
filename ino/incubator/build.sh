#platformio run -e arduino_uno -v &>build.log
platformio run -e esp01 -v &>build.log
echo $?
