# Lanzar los controladores en terminales separados
setsid ros2 run px4_ros_com offboard_control_multi --ros-args -r __ns:=/px4_1 > uav1.log 2>&1 &
sleep 2
setsid ros2 run px4_ros_com offboard_control_multi --ros-args -r __ns:=/px4_2 > uav2.log 2>&1 &
sleep 2