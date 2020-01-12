export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

platformio run -t upload -v -e stm8sblue

echo $?
