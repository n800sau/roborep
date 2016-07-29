ffmpeg -framerate 1 -pattern_type glob -i 'output/*.jpg' -c:v libx264 -r 1 output.mp4
