cd src/data && \
for file in `ls -A1`; do curl -F "file=@$PWD/$file" webservos.local/edit; done
