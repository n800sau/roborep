#echo Hello | festival --tts --pipe
echo Hello | text2wave -f 44100 |aplay
echo $?
