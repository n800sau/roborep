BASE_URL=http://192.168.1.89
USERCRED=admin:admin
cd src/data && \
for file in `ls -A1 *`
do
#	echo Delete $file
#	curl -v -u "$USERCRED" --digest -X DELETE "$BASE_URL/edit?path=$file"
	echo Upload $file
	curl -u "$USERCRED" --digest --fail -F "data=@$file;filename=/$file" "$BASE_URL/edit" && echo && echo $file Done || (echo $? && exit 1) || break
done
