rosrun xacro xacro --inorder oculus_robo.xacro > oculus_robo.urdf 2> mk_urdf.err && \
check_urdf oculus_robo.urdf &> check_urdf.log
echo $?
