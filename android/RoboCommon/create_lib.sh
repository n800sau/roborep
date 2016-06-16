export PATH=$PATH:/home/n800s/work/android-sdk-linux/tools
android create lib-project --target 11 --name RoboCommon --path . --package au.n800s.robo.common &> create.log
echo $?
