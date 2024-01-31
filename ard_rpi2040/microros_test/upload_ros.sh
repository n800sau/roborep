if [ -z "$1" ]
then
	BIN=build/pico_micro_ros_example.elf
else
	BIN="$1"
fi

gdb-multiarch -q --command=upload.gdb --args "$BIN"
