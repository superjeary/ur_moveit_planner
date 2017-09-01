# ur_moveit_planner
moveit planner for universal robot ur5

change spawn position in universal_robot/ur_gazebo/launch/ur5.launch (ex -z 1.0) 
<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -z 1" respawn="false" output="screen" />
