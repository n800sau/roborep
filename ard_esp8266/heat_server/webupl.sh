cd src/data && \
for file in `ls -A1 *`
do
#	echo Delete $file
#	curl -v -u admin:parol --digest -X DELETE -F "data=@$PWD/$file" --fail "http://heat-server.local/edit"
	echo Upload $file
	curl -u admin:parol --digest --fail -F "data=@$file;filename=/$file" "http://heat-server.local/edit" && echo && echo $file Done || (echo $? && exit 1) || break
#	curl -v -u admin:parol --digest --fail --upload-file "$file" --data-urlencode path=$file "http://heat-server.local/edit" && echo $file || (echo $? && exit 1) || break
done
