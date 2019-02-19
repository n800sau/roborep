cd src/data && \
for file in `ls -A1`
do
	curl -F "file=@$PWD/$file" pcr_server.local/edit && echo $file || (echo $? && exit 1) || break
done
