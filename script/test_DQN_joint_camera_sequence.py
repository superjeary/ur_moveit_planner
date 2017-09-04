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

from DQN_joint_camera_sequence import robotEnv, QFunction, RandomActor

rospy.init_node('test_DQN_joint')
print("Test start.")
env = robotEnv()
ra = RandomActor(env)
n_history = env.n_history
n_actions = env.actionNum
q_func = QFunction(n_history, n_actions)
q_func.to_gpu(0) ## GPUを使いたい人はこのコメントを外す
optimizer = chainer.optimizers.Adam(eps=1e-2)
optimizer.setup(q_func)
# 報酬の割引率
gamma = 0.95
# Epsilon-greedyを使ってたまに冒険。50000ステップでend_epsilonとなる
#explorer = chainerrl.explorers.LinearDecayEpsilonGreedy(
#    start_epsilon=1.0, end_epsilon=0.3, decay_steps=1000, random_action_func=ra.random_action_func)
explorer = chainerrl.explorers.ConstantEpsilonGreedy(
    epsilon=0.3, random_action_func=ra.random_action_func)
# Experience ReplayというDQNで用いる学習手法で使うバッファ
replay_buffer = chainerrl.replay_buffer.ReplayBuffer(capacity=10 ** 6)
# Agentの生成（replay_buffer等を共有する2つ）
agent = chainerrl.agents.DoubleDQN(
    q_func, optimizer, replay_buffer, gamma, explorer,
    replay_start_size=32, update_interval=1,
    target_update_interval=10)
agent.load("result_joint_camera_sequence/resultDQN_joint_100")

env.reset()

while not rospy.is_shutdown():
    env.set_state(env.image_resize)
    action = agent.act(env.state)
    if not env.move_action(action):
        "Problem is occured!"
        quit()
    elif env.timeOver == True:
        print("Time Over!!")
        break
print("init area", env.initArea)
print("result area", env.area)
print("")
print("Test finished.")