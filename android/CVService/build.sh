ant debug &>build.log && \
cp bin/CVService-debug.apk ~/public_html/android
echo $?
#ant debug && adb install -r bin/RoboSensorView-debug.apk

#ant release

