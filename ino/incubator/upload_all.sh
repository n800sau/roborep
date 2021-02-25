#cd src/data && for file in `ls -A1`; do curl -vvv --digest -F "data=@$file" -F "filename=/$file" --user admin:admin --trace-ascii ../../trace.txt http://192.168.1.89/edit; done && cd -
#cd src/data && for file in `ls -A1`; do http -A digest -a admin:admin pie.dev/digest-auth/httpie/username/password -f POST pie.dev/post data@"$file" http://192.168.1.89/edit; done && cd -
cd src/data && for file in `ls -A1`; do curl -v --user admin:admin --digest -X PUT -T "$file" "http://192.168.1.89/edit?path=/$file"; done && cd -

