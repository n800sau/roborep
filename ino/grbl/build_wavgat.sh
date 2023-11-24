source vars.sh
arduino-cli compile --fqbn "$BOARD" grblUpload
#arduino-cli upload -p "$DEV" --fqbn "$BOARD" grblUpload
