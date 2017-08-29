#include "ur_moveit_planner.h"
#include <tf/transform_listener.h>    // Includes the TF conversions

UR_Moveit_Planning::UR_Moveit_Planning() : group("manipulator"), moveToCartesianPoseActionServer_(n_, 
    "ur_moveit_planner/moveToCartesianPoseAction", boost::bind(&UR_Moveit_Planning::execute, this, _1),false)
{
  ROS_INFO_STREAM("Constructor of UR_Moveit_Planning class that contains movegroup has started.");

  moveToCartesianPoseService = n_.advertiseService("ur_moveit_planner/moveToCartesianPose", &UR_Moveit_Planning::moveToCartesianPoseCallback, this);
  moveToJointAnglesService = n_.advertiseService("ur_moveit_planner/moveToJointAnglesPose", &UR_Moveit_Planning::moveToJointAnglesCallback, this);
  stopMovingService = n_.advertiseService("ur_moveit_planner/stopMoving", &UR_Moveit_Planning::stopMovingCallback, this);

  // Configure movegroup planners
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink("ee_link");

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveToCartesianPoseActionServer_.start();

}

bool UR_Moveit_Planning::moveToCartesianPose(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroup &group)
{
  geometry_msgs::PoseStamped command_cartesian_position;
  bool success_plan = false, motion_done = false, new_pose = false;
  while (ros::ok()) {     
    command_cartesian_position = group.getCurrentPose(target_ee_link);  
    command_cartesian_position.pose = target_pose;
    
    group.setStartStateToCurrentState();
    group.setPoseTarget(command_cartesian_position, target_ee_link);
    success_plan = group.plan(myplan);
    if (success_plan) 
    {
       motion_done = group.execute(myplan);
    } 
    else if (!success_plan) 
    {
       ROS_ERROR_STREAM("No path to the target point has been found!");
       return false;
    }
    if (motion_done) 
    {
       break;
    }
  }
  return true;
  
}

bool UR_Moveit_Planning::moveToCartesianPoseCallback(ur_moveit_planner::moveToCartesianPose::Request  &req,
         ur_moveit_planner::moveToCartesianPose::Response &res)
{
  ROS_INFO_STREAM("Entered moveToCartesianPose service.");
  return moveToCartesianPose(req.target_pose, group);
}

bool UR_Moveit_Planning::moveToJointAngles(const double& a1, const double& a2, const double& a3, const double& a4, const double& a5, const double& a6, moveit::planning_interface::MoveGroup &group)
{
  std::vector<double> command_joint_angles_values;
  bool success_plan = false, motion_done = false, new_pose = false;
  while (ros::ok()) {
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), command_joint_angles_values);
    command_joint_angles_values[0] = a1;
    command_joint_angles_values[1] = a2;
    command_joint_angles_values[2] = a3;
    command_joint_angles_values[3] = a4;
    command_joint_angles_values[4] = a5;
    command_joint_angles_values[5] = a6;
 
    group.setJointValueTarget(command_joint_angles_values);
    success_plan = group.plan(myplan);
    if (success_plan) 
    {
       ROS_INFO_STREAM("robot move");
       //motion_done = group.execute(myplan);
       motion_done = group.move();
    } 
    else if (!success_plan) 
    {
       ROS_ERROR_STREAM("No path to the target point has been found!");
       return false;
    }
    if (motion_done) 
    {
       break;
    }
  }
  return true;
  
}

bool UR_Moveit_Planning::moveToJointAnglesCallback(ur_moveit_planner::moveToJointAngles::Request  &req,
         ur_moveit_planner::moveToJointAngles::Response &res)
{
  ROS_INFO_STREAM("Entered moveToJointAngles service.");
  return moveToJointAngles(req.a1, req.a2, req.a3, req.a4, req.a5, req.a6, group);
  
}

bool UR_Moveit_Planning::stopMovingCallback(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  group.stop();
  return true;
}

void UR_Moveit_Planning::execute(const ur_moveit_planner::moveToCartesianPoseGoalConstPtr& goal)
{
  ROS_INFO("moveToCartesitnPoseAction was called");
  geometry_msgs::Point p;
  p.x = goal->x;
  p.y = goal->y;
  p.z = goal->z;

  geometry_msgs::Quaternion q;
  tf::Quaternion qt;
  qt.setRPY(goal->u, goal->v, goal->w);
  tf::quaternionTFToMsg(qt, q);

  // Create the pose
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;
  moveToCartesianPose(pose, group);
  //std::this_thread::sleep_for(std::chrono::milliseconds(200)); //default 2000
  ROS_INFO("moveToCartPosePTPAction is set as succeeded");
  moveToCartesianPoseActionServer_.setSucceeded();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_moveit_planner");
  ROS_INFO("ur_moveit_planner node is starting up");
  // Without the AsyncSpinner, the iiwa_ros object does not seem to connect
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ros::NodeHandle n;
  //Create an object of class UR_Moveit_Planning that will take care of everything
  UR_Moveit_Planning URObject;

  

  ROS_INFO("ur_moveit_planner node has started");

  ros::spin();
  return 0;
}

