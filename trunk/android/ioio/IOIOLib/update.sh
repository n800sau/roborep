PWD=`pwd`
NAME=`basename $PWD`
echo $NAME
android update project -p . --name $NAME --target 4 --subprojects

