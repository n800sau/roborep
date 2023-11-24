while true
do
	rostopic echo -n1 /opcplus/lwheel_desired_rate > m_rate.tmp && \
	rostopic echo -n1 /opcplus/rwheel_desired_rate >> m_rate.tmp && \
	mv m_rate.tmp m_rate.msg
done

