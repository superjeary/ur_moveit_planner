#include "ur_moveit_planner.h"
#include <tf/transform_listener.h>    // Includes the TF conversions

//#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/Point.h"
//#include "geometry_msgs/Quaternion.h"
#include "ur_moveit_planner/JointQuantity.h"
#include "ur_moveit_planner/PoseRpy.h"

UR_Moveit_Planning::UR_Moveit_Planning() : group("manipulator"), moveToCartesianPoseActionServer_(n_, 
    "ur_moveit_planner/moveToCartesianPoseAction", boost::bind(&UR_Moveit_Planning::executeCartesianPose, this, _1),false),
    moveToJointAnglesActionServer_(n_, 
      "ur_moveit_planner/moveToJointAnglesAction", boost::bind(&UR_Moveit_Planning::executeJointAngles, this, _1),false)
{
  ROS_INFO_STREAM("Constructor of UR_Moveit_Planning class that contains movegroup has started.");

  moveToCartesianPoseService = n_.advertiseService("ur_moveit_planner/moveToCartesianPose", &UR_Moveit_Planning::moveToCartesianPoseCallback, this);
  moveToJointAnglesService = n_.advertiseService("ur_moveit_planner/moveToJointAngles", &UR_Moveit_Planning::moveToJointAnglesCallback, this);
  stopMovingService = n_.advertiseService("ur_moveit_planner/stopMoving", &UR_Moveit_Planning::stopMovingCallback, this);
  //getCurrentPoseService = n_.advertiseService("ur_moveit_planner/getCurrentPose", &UR_Moveit_Planning::getCurrentPoseCallback, this);
  //getCurrentJointAnglesService = n_.advertiseService("ur_moveit_planner/getJointAngles", &UR_Moveit_Planning::getCurrentJointAnglesCallback, this);

  // Configure movegroup planners
  group.setPlanningTime(0.5);
  //group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink("ee_link");

  /*moveit_msgs::JointConstraint ocm;
  ocm.joint_name = "shoulder_pan_joint";
  ocm.position = 0.0;
  ocm.tolerance_above = 90.0/180.0*3.14;
  ocm.tolerance_below = 90.0/180.0*3.14;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.joint_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);*/

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveToCartesianPoseActionServer_.start();
  moveToJointAnglesActionServer_.start();

  _pubCurrentPose = n_.advertise<geometry_msgs::PoseStamped>("ur_moveit_planner/currentPose", 1);
  _pubCurrentPoseRpy = n_.advertise<ur_moveit_planner::PoseRpy>("ur_moveit_planner/currentPoseRpy", 1);
  _pubCurrentJointAngles = n_.advertise<ur_moveit_planner::JointQuantity>("ur_moveit_planner/currentJointAngles", 1);
  

  while(ros::ok()){
    geometry_msgs::PoseStamped currentPose = group.getCurrentPose();
    _pubCurrentPose.publish(currentPose);

    ur_moveit_planner::PoseRpy currentPoseRpy;
    currentPoseRpy.position = currentPose.pose.position;

    double r,p,y;//出力値
    tf::Quaternion quat(currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z,currentPose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);//クォータニオン→オイラー角
    currentPoseRpy.rpy.u = r;
    currentPoseRpy.rpy.v = p;
    currentPoseRpy.rpy.w = y;
    _pubCurrentPoseRpy.publish(currentPoseRpy);

    std::vector<double> currentJointValues = group.getCurrentJointValues();
    ur_moveit_planner::JointQuantity currentJointAngles;
    currentJointAngles.a1 = currentJointValues[0];
    currentJointAngles.a2 = currentJointValues[1];
    currentJointAngles.a3 = currentJointValues[2];
    currentJointAngles.a4 = currentJointValues[3];
    currentJointAngles.a5 = currentJointValues[4];
    currentJointAngles.a6 = currentJointValues[5];
    _pubCurrentJointAngles.publish(currentJointAngles);

    ros::spinOnce();
  }

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

bool UR_Moveit_Planning::moveToCartesianPoseLIN(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroup &group)
{
  geometry_msgs::Pose command_cartesian_position;
  std::vector<geometry_msgs::Pose> waypoints;
  //group.setMaxVelocityScalingFactor(0.1);
  bool success_plan = false, motion_done = false, new_pose = false;
  while (ros::ok()) {     
    command_cartesian_position = target_pose;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
    group.setStartStateToCurrentState();
    int point_num = trajectory.joint_trajectory.points.size();
    for(int i=0;i < point_num;i++){
      std::vector<double> joint_value = trajectory.joint_trajectory.points[i].positions;
      group.setJointValueTarget(joint_value);
    }
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
  //return moveToCartesianPose(req.target_pose, group);
  return moveToCartesianPoseLIN(req.target_pose, group);
}

bool UR_Moveit_Planning::moveToJointAngles(const double& a1, const double& a2, const double& a3, const double& a4, const double& a5, const double& a6, moveit::planning_interface::MoveGroup &group)
{
  std::vector<double> command_joint_angles_values;
  bool success_plan = false, motion_done = false;
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

/*bool UR_Moveit_Planning::getCurrentPoseCallback(std_srvs::Empty::Request  &req,
  std_srvs::Empty::Response &res)
{
  //_pubCurrentPose = n_.advertise<geometry_msgs::PoseStamped>("ur_moveit_planner/currentPose", 1);
  
  if(ros::ok()){
    geometry_msgs::PoseStamped currentPose = group.getCurrentPose();
    _pubCurrentPose.publish(currentPose);
    ros::spinOnce();
    
    return true;
  }

    return false;
}*/

void UR_Moveit_Planning::executeCartesianPose(const ur_moveit_planner::moveToCartesianPoseGoalConstPtr& goal)
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

void UR_Moveit_Planning::executeJointAngles(const ur_moveit_planner::moveToJointAnglesGoalConstPtr& goal)
{
  ROS_INFO("moveToJointAnlgesAction was called");
  
  moveToJointAngles(goal->a1, goal->a2, goal->a3, goal->a4, goal->a5, goal->a6, group);
  //std::this_thread::sleep_for(std::chrono::milliseconds(200)); //default 2000
  ROS_INFO("moveToJointAnglesPTPAction is set as succeeded");
  moveToJointAnglesActionServer_.setSucceeded();
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

