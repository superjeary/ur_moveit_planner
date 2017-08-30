# -*- coding: utf-8 -*-
import chainer
import chainer.functions as F
import chainer.links as L
import chainerrl
import numpy as np
import random
import copy
import math

import rospy
import actionlib
from ur_moveit_planner.msg import moveToCartesianPoseAction
from ur_moveit_planner.msg import moveToCartesianPoseGoal
from ur_moveit_planner.msg import moveToJointAnglesAction
from ur_moveit_planner.msg import moveToJointAnglesGoal

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from ur_moveit_planner.msg import PoseRpy,JointQuantity

from DQN_proto_joint import robotAgent, robotEnv, QFunction, RandomActor

rospy.init_node('test_DQN')
print("Training start.")
env = robotEnv()
ra = RandomActor(env)
obs_size = 6
n_actions = 4
q_func = QFunction(obs_size, n_actions)
q_func.to_gpu(0) ## GPUを使いたい人はこのコメントを外す
optimizer = chainer.optimizers.Adam(eps=1e-2)
optimizer.setup(q_func)
# 報酬の割引率
gamma = 0.95
# Epsilon-greedyを使ってたまに冒険。50000ステップでend_epsilonとなる
explorer = chainerrl.explorers.LinearDecayEpsilonGreedy(
    start_epsilon=1.0, end_epsilon=0.3, decay_steps=1000, random_action_func=ra.random_action_func)
# Experience ReplayというDQNで用いる学習手法で使うバッファ
replay_buffer = chainerrl.replay_buffer.ReplayBuffer(capacity=10 ** 6)
# Agentの生成（replay_buffer等を共有する2つ）
agent = chainerrl.agents.DoubleDQN(
    q_func, optimizer, replay_buffer, gamma, explorer,
    replay_start_size=500, update_interval=1,
    target_update_interval=100)
agent.load("resultDQN_pose_100")

for i in range(10):
    env.reset()
    
    while not rospy.is_shutdown():
        action = agent.act(env.robotState.copy())
        if not env.move_action(action):
            "Problem is occured!"
            quit()
        if env.done == True:
            break
        elif env.timeOver == True:
            print("Time Over!!")
            break
    print("try " + str(i))
    print("init distance", env.initDistance)
    print("result Distance", env.distance)
    print("")
print("Test finished.")