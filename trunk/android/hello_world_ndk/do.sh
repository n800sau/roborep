# create tool-chain - one line
$NDK_HOME/build/tools/make-standalone-toolchain.sh --platform=android-8 --install-dir=/tmp/my-android-toolchain

if [ $? == 0 ]
then
	# add to terminal PATH variable
	export PATH=/tmp/my-android-toolchain/bin:$PATH

	# make alias CC be the new gcc binary
	export CC=arm-linux-androideabi-gcc

	# compile your C code(I tried hello world)
	$CC -o hello hello.c

	if [ $? == 0 ]
	then
		# push binary to phone
		adb push hello /data/local/tmp

		# execute binary
		adb shell chmod 777 /data/local/tmp/hello

		adb shell /data/local/tmp/hello
	fi
fi

