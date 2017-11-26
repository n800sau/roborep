rosrun xacro xacro --inorder bwh_robo.xacro > bwh_robo.urdf 2> mk_urdf.err && \
check_urdf bwh_robo.urdf &> check_urdf.log
echo $?
