#platformio run -e arduino_uno -v &>build.log
platformio run -e esp -v &>build.log
echo $?
