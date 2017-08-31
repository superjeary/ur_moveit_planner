#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math

import actionlib
from ur_moveit_planner.msg import moveToJointAnglesAction
from ur_moveit_planner.msg import moveToJointAnglesGoal

if __name__ == '__main__':
    rospy.init_node('action_client_joint')
    action_client = actionlib.SimpleActionClient("ur_moveit_planner/moveToJointAnglesAction",moveToJointAnglesAction)
    action_client.wait_for_server()
    rospy.loginfo("server ON")

    target_jointAngles = moveToJointAnglesGoal()
    target_jointAngles.a1 = 0.0 /180*math.pi
    target_jointAngles.a2 = -90.0 /180*math.pi
    target_jointAngles.a3 = 90.0 /180*math.pi
    target_jointAngles.a4 = -90.0 /180*math.pi
    target_jointAngles.a5 = -90.0 /180*math.pi
    target_jointAngles.a6 = 90.0 /180*math.pi

    action_client.send_goal(target_jointAngles)
    finished = action_client.wait_for_result(rospy.Duration(5.0))
    if finished:
        result = action_client.get_state();
        print result
    else:
        print "time out!!"