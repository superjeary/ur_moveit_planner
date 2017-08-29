#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from geometry_msgs.msg import Pose, Quaternion
from ur_moveit_planner.srv import moveToCartesianPose
import copy
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool, Empty
import math
import time
import tf

class demo:
    def __init__(self):
        rospy.loginfo("wait for service...")
        rospy.wait_for_service('ur_moveit_planner/moveToCartesianPose')
	
        try:
            self.moveit_service = rospy.ServiceProxy('ur_moveit_planner/moveToCartesianPose', moveToCartesianPose)
        except rospy.ServiceException, e:
            rospy.logerr("Moveit Planner Service call failed: %s" % e)
        rospy.loginfo("start ur_moveit_planner/moveToCartesianPose service")

        rospy.wait_for_service('ur_moveit_planner/stopMoving')
        try:
            self.stop_service = rospy.ServiceProxy('ur_moveit_planner/stopMoving', Empty)
        except rospy.ServiceException, e:
            rospy.logerr("Stop Moving Service call failed: %s" % e)
        rospy.loginfo("start ur_moveit_planner/stopMoving service")

        """
        try:
            self.suction_service = rospy.ServiceProxy('suction_switch', SetBool)
        except rospy.ServiceException, e:
            rospy.logerr("Suction Service call failed: %s" % e)
        """

        rospy.loginfo("init")

    def initialize(self):
        rospy.loginfo("initialize")
        #self.call_suction_switch(False)

        self.init_pose = Pose()
        self.init_pose.position.x = 0.47293
        self.init_pose.position.y = -0.01787
        self.init_pose.position.z = 0.68797
        roll= 179.9 /180*math.pi
        pitch = 0     /180*math.pi
        yaw = -180  /180*math.pi

        self.init_pose.orientation = self.e2q(roll, pitch, yaw)
        
        self.call_ur_moveit_planner(self.init_pose)
        #if(self.call_ur_moveit_planner(self.init_pose)):
        #    print("Initialized")

    def e2q(self, roll, pitch, yaw):
	    q = Quaternion()
        quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        q.x = quaternion[0]
        q.y = quaternion[1]
        q.z = quaternion[2]
        q.w = quaternion[3]
        
        return q

    def call_ur_moveit_planner(self,targetPose):
        response = self.moveit_service(targetPose)
        rospy.loginfo(response)

    def call_suction_service(self,switch):
        response = self.suction_service(switch)
        rospy.loginfo(response)

    def call_stop_moving(self):
        response = self.stop_service()
        
if __name__ == "__main__":
    rospy.init_node('ur_demo')

    
    d = demo()
    d.initialize()

    d.call_stop_moving()
