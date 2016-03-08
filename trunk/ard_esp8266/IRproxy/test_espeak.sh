#espeak "Hello, I am Espeak, the voice synthesizer" --stdout | paplay -v
#espeak -v en "Hello i am espeak" --stdout | aplay -v
#espeak -v en "Hello i am espeak" --stdout | sndfile-resample -to 44100 /dev/stdin /dev/stdout > hello.wav
espeak -v en "Hello i am espeak" --stdout|sox -t wav - -r 44100 -t wav - | aplay -v
