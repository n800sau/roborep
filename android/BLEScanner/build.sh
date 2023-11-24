./gradlew assemble &>build.log && \
cp app/build/outputs/apk/app-debug.apk ~/public_html/android/btscanner.apk
echo $?
