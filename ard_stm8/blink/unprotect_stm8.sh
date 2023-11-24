echo "00 00 ff 00 ff 00 ff 00 ff 00 ff" | xxd -r -p > factory_defaults.bin
~/.platformio/packages/tool-stm8tools/stm8flash -c stlinkv2 -p stm8s103f3 -s opt -w factory_defaults.bin
