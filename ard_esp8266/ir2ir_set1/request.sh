fname=r2.json
wget -q --post-file "$fname" -O response.json http://ir2ir.local/
echo $?
