#ino build &>build.log


ino build &>build.log && \
ino upload &> upload.log
echo $?

#ino build && ./upload.sh
