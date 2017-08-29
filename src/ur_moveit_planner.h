#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "std_srvs/Empty.h"
#include "ur_moveit_planner/moveToCartesianPose.h"
#include "ur_moveit_planner/moveToJointAngles.h"
#include "ur_moveit_planner/currentPose.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "ur_moveit_planner/moveToCartesianPoseAction.h"

class UR_Moveit_Planning
{
public:
  //Constructor
  UR_Moveit_Planning();

  bool initializePositions(); // Populates joint angles/positions we use. Called during startup.

  bool moveToCartesianPose(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroup &group);
  bool moveToCartesianPoseCallback(ur_moveit_planner::moveToCartesianPose::Request &req, ur_moveit_planner::moveToCartesianPose::Response &res);

  bool moveToJointAngles(const double& a1, const double& a2, const double& a3, const double& a4, const double& a5, const double& a6, moveit::planning_interface::MoveGroup &group);
  bool moveToJointAnglesCallback(ur_moveit_planner::moveToJointAngles::Request &req, ur_moveit_planner::moveToJointAngles::Response &res);

  bool stopMovingCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  //bool getCurrentPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  //bool getCurrentJointAnglesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void execute(const ur_moveit_planner::moveToCartesianPoseGoalConstPtr& goal);

private:
  ros::NodeHandle n_;

  ros::ServiceServer moveToCartesianPoseService;
  ros::ServiceServer moveToJointAnglesService;
  ros::ServiceServer stopMovingService;
  ros::ServiceServer getCurrentPoseService;
  ros::ServiceServer getCurrentJointAnglesService;
  

  std::string target_ee_link;

  // MoveGroup for planning
  moveit::planning_interface::MoveGroup group;
  moveit::planning_interface::MoveGroup::Plan myplan;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  actionlib::SimpleActionServer<ur_moveit_planner::moveToCartesianPoseAction> moveToCartesianPoseActionServer_; 

  ros::Publisher _pubCurrentPose;
  ros::Publisher _pubCurrentJointAngles;

  // tf::TransformListener tflistener;
};

