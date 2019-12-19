cd src/data && \
for file in `ls -A1`; do curl -F "file=@$PWD/$file" web2ir.local/edit; done
