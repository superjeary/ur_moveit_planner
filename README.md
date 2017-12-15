# ur_moveit_planner
moveit planner server for universal robot ur5

# Preparation
Check out or install these packages on your catkin workspace

## Indogo
```
cd ~/(YOUR_CATKIN_WORKSPACE)/src
sudo apt-get install ros-indigo-opencv3
git clone https://github.com/ros-industrial/universal_robot
git clone https://github.com/ThomasTimm/ur_modern_driver
git clone https://github.com/yupos0221/ur_moveit_planner.git
```

## kinetic
```
cd (YOUR_CATKIN_WORKSPACE)/src
sudo apt-get install ros-kinetic-opencv3
git clone https://github.com/ros-industrial/universal_robot
git clone -b iron-kinetic-devel https://github.com/iron-ox/ur_modern_driver
git clone https://github.com/yupos0221/ur_moveit_planner.git
```

Change ur_moveit_planner/CMakeLists.txt to compile for kinetic

```
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -2,7 +2,7 @@ cmake_minimum_required(VERSION 2.8.3)
     project(ur_moveit_planner)
     
     ## Compile as C++11, supported in ROS Kinetic and newer
    -# add_compile_options(-std=c++11)
    +add_compile_options(-std=c++11)
```

# Install
```
cd (YOUR_CATKIN_WORKSPACE)
catkin_make
```


# How to use with real hardware
## 1. Bring up UR5 and start-up server
```
roslaunch ur_moveit_planner ur_robot.launch
```

## 2. Running the sample (in new terminal)
!CAUTION! Please check robot trajectry and your safe.
```
roscd ur_moveit_planner/script
python action_client.py
```

# How to use with simulation
## 1. Bring up gazebo and start-up server
```
roslaunch ur_moveit_planner ur_gazebo.launch
```

## 2. Running the sample (in new terminal)
```
roscd ur_moveit_planner/script
python action_client.py
```
