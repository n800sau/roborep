shasum -b -a 384 $1  | awk '{ print $1 }' | xxd -r -p | base64
