BASE_URL=http://192.168.1.89
cd src/data && \
for file in `ls -A1 *`
do
#	echo Delete $file
#	curl -v -u admin:parol --digest -X DELETE -F "data=@$PWD/$file" --fail "http://$BASE_URL/edit"
	echo Upload $file
	curl -u admin:admin --digest --fail -F "data=@$file;filename=/$file" "$BASE_URL/edit" && echo && echo $file Done || (echo $? && exit 1) || break
#	curl -v -u admin:parol --digest --fail --upload-file "$file" --data-urlencode path=$file "$BASE_URL/edit" && echo $file || (echo $? && exit 1) || break
done
