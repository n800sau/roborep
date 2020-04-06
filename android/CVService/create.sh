export PATH=$PATH:/home/n800s/work/android-sdk-linux/tools
android create project --target 11 --name CVService --path . --activity CameraActivity --package au.n800s.cv &> create.log
echo $?
