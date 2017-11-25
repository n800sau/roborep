while true
do
	rostopic echo -n1 /opcplus/scan > m_scan.tmp && \
	mv m_scan.tmp m_scan.msg
done

