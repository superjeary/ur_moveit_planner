#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

import actionlib
from ur_moveit_planner.msg import moveToCartesianPoseAction
from ur_moveit_planner.msg import moveToCartesianPoseGoal

if __name__ == '__main__':
    rospy.init_node('action_client')
    action_client = actionlib.SimpleActionClient("ur_moveit_planner/moveToCartesianPoseAction",moveToCartesianPoseAction)
    action_client.wait_for_server()
    rospy.loginfo("server ON")

    target_pose = moveToCartesianPoseGoal()

    target_pose.x = 0.448;
    target_pose.y = 0.0;
    target_pose.z = 0.6505;
    target_pose.u = 3.14;
    target_pose.v = 0;
    target_pose.w = 3.14;

    action_client.send_goal(target_pose)
    finished = action_client.wait_for_result(rospy.Duration(5.0))
    if finished:
        result = action_client.get_state();
        print result
    else:
        print "time out!!"