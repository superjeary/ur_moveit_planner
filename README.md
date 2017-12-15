# ur_moveit_planner
moveit planner server for universal robot ur5

# Preparation
Check out these packages on your catkin workspace

https://github.com/ros-industrial/universal_robot

https://github.com/ThomasTimm/ur_modern_driver

# install
```
cd ~/(YOUR_CATKIN_WORKSPACE)/src
git clone https://github.com/yupos0221/ur_moveit_planner.git
cd ../
catkin_make
```


# How to use with real hardware
## 1. Bring up UR5 and startup server
```
roslaunch ur_moveit_plannner ur_robot.launch
```

## 2. Running the sample (in new terminal)
!CAUTION! Please check robot trajectry and your safe.
```
roscd ur_moveit_planner/script
python action_client.py
```

# How to use with simulation
## 1. Bring up gazebo and startup server
```
roslaunch ur_moveit_plannner ur_gazebo.launch
```

## 2. Running the sample (in new terminal)
```
roscd ur_moveit_planner/script
python action_client.py
```
