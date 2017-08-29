#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Pose
from ur_moveit_planner.srv import moveToCartesianPose
import tf
import math

class demo:
    def __init__(self):
        rospy.loginfo("wait for service...")
        rospy.wait_for_service('ur_moveit_planner/moveToCartesianPose')
        rospy.loginfo("init")

    def call_ur_moveit_planner(self,targetPose):
        try:
            service = rospy.ServiceProxy('ur_moveit_planner/moveToCartesianPose', moveToCartesianPose)
            response = service(targetPose)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == "__main__":
    rospy.init_node("ur_moveit_planner_clinet")

    d = demo()

    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.0
    pose.position.z = 0.5

    roll = 0.0 /180*math.pi
    pitch = 90.0 /180*math.pi
    yaw = 0.0 /180*math.pi

    quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    
    d.call_ur_moveit_planner(pose)

    print("end")
