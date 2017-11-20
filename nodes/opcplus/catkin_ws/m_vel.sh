while true
do
	rostopic echo -n1 /opcplus/cmd_vel > m_vel.tmp && \
	mv m_vel.tmp m_vel.msg
done

