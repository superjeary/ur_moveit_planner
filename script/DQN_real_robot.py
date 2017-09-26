# -*- coding: utf-8 -*-
import chainer
import chainer.functions as F
import chainer.links as L
import chainerrl
import numpy as np
import random
import copy
import math
import sys

import rospy
import rospkg
import actionlib
from ur_moveit_planner.msg import moveToCartesianPoseAction
from ur_moveit_planner.msg import moveToCartesianPoseGoal
from ur_moveit_planner.msg import moveToJointAnglesAction
from ur_moveit_planner.msg import moveToJointAnglesGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from ur_moveit_planner.msg import PoseRpy,JointQuantity

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class QFunction(chainer.Chain):
    def __init__(self, n_history, n_act):
        super(QFunction, self).__init__(
            l1=L.Convolution2D(n_history, 32, ksize=8, stride=4, nobias=False,initialW=chainer.initializers.Normal(np.sqrt(2) / math.sqrt(10))),
            l2=L.Convolution2D(32, 64, ksize=4, stride=2, nobias=False,initialW=chainer.initializers.Normal(np.sqrt(2) / math.sqrt(10))),
            l3=L.Convolution2D(64, 64, ksize=3, stride=1, nobias=False, initialW=chainer.initializers.Normal(np.sqrt(2) / math.sqrt(10))),
            l4=L.Linear(3136, 512, initialW=chainer.initializers.Normal(np.sqrt(2) / math.sqrt(10))),
            q_value=L.Linear(512, n_act, initialW=np.zeros((n_act, 512), dtype=np.float32))
        )

    def __call__(self, state, test=False):
        h1 = F.leaky_relu(self.l1(state/255.))
        h2 = F.leaky_relu(self.l2(h1))
        h3 = F.leaky_relu(self.l3(h2))
        h4 = F.leaky_relu(self.l4(h3))
        return chainerrl.action_value.DiscreteActionValue(self.q_value(h4))
    
    """
    def __call__(self, state, test=False):
        h1 = F.relu(self.l1(state/255.))
        h2 = F.relu(self.l2(h1))
        h3 = F.relu(self.l3(h2))
        h4 = F.relu(self.l4(h3))
        return chainerrl.action_value.DiscreteActionValue(self.q_value(h4))
    """

class RandomActor:
    def __init__(self,env):
        self.env = env
        self.random_count = 0
    def random_action_func(self):
        self.random_count += 1
        return random.randint(0,env.actionNum-1)


# Define Environment
class robotEnv:
    def __init__(self):
        self.n_history = 4
        self.rotateUnit = 3.14/72
        self.actionNum = 8
        self.actions = np.array([
        [self.rotateUnit, 0.0, 0.0, 0.0, 0.0, 0.0], [-self.rotateUnit, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, self.rotateUnit, 0.0, 0.0, 0.0, 0.0], [0.0, -self.rotateUnit, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, self.rotateUnit, 0.0, 0.0, 0.0], [0.0, 0.0, -self.rotateUnit, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, self.rotateUnit, 0.0, 0.0], [0.0, 0.0, 0.0, -self.rotateUnit, 0.0, 0.0],
        #[0.0, 0.0, 0.0, 0.0, self.rotateUnit, 0.0], [0.0, 0.0, 0.0, 0.0, -self.rotateUnit, 0.0],
        #[0.0, 0.0, 0.0, 0.0, 0.0, self.rotateUnit], [0.0, 0.0, 0.0, 0.0, 0.0, -self.rotateUnit]
        ]) 

        self.robotJointAngles = moveToJointAnglesGoal()
       
        self.TCPForce = 0
        self.done = False
        self.initJointAngles = moveToJointAnglesGoal()
        """
        self.initJointAngles.a1 = 0.0 /180*math.pi
        self.initJointAngles.a2 = -90.0 /180*math.pi
        self.initJointAngles.a3 = 90.0 /180*math.pi
        self.initJointAngles.a4 = -90.0 /180*math.pi
        self.initJointAngles.a5 = -90.0 /180*math.pi
        self.initJointAngles.a6 = 90.0 /180*math.pi
        """

        self.initJointAngles.a1 = -120.0 /180*math.pi
        self.initJointAngles.a2 = -90.0 /180*math.pi
        self.initJointAngles.a3 = -90.0 /180*math.pi
        self.initJointAngles.a4 = -90.0 /180*math.pi
        self.initJointAngles.a5 = 90.0 /180*math.pi
        self.initJointAngles.a6 = 0 /180*math.pi

        self.initPose = moveToCartesianPoseGoal()
        self.initPose.x = 0.4869
        self.initPose.y = 0.1090
        self.initPose.z = 0.5140
        self.initPose.u = -90.0 /180*math.pi
        self.initPose.v = 0.0 /180*math.pi
        self.initPose.w = 180.0 /180*math.pi

        self.initArea = 0
        self.area = 0

        self.startTime = rospy.get_time()
        self.timeOver = False
        
        #FOR Camera
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/gazebo/camera1/image_raw",Image,self.cameraCallback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.cameraCallback)
        self.image_raw = None
        self.image_resize = None

        rospy.loginfo("waiting for server ...")
        _jointAnglesSub = rospy.Subscriber('/ur_moveit_planner/currentJointAngles', JointQuantity, self.callbackJointAngles)
        self.action_client = actionlib.SimpleActionClient("ur_moveit_planner/moveToJointAnglesAction",moveToJointAnglesAction)
        self.action_client.wait_for_server()
        rospy.loginfo("server ON")

        self.reset()

    def reset(self):
        self.move(self.initJointAngles)
        self.TCPForce = 0
        self.done = False
        self.startTime = rospy.get_time()
        self.timeOver = False
        self.initArea = self.area
        print(self.initArea)
        self.reset_state(self.image_resize)
    
    def move(self, targetJointAngles, train=True):
        self.action_client.send_goal(targetJointAngles)
        if(train):
            finished = self.action_client.wait_for_result(rospy.Duration(5.0))
            if finished:
                result = self.action_client.get_state();
                self.checkScore()
                #self.done = True
                return True
            else:
                print "time out!!"
                return False
        else:
            self.checkScore()

    def move_action(self, act, train=True):
        action = self.actions[act]
        
        #print(act, action, self.robotJointAngles)
        nextRobotJointAngles = moveToJointAnglesGoal()
        currentRobotJointAngles = copy.deepcopy(self.robotJointAngles)
        
        nextRobotJointAngles.a1 = currentRobotJointAngles.a1 + action[0]
        nextRobotJointAngles.a2 = currentRobotJointAngles.a2 + action[1]
        nextRobotJointAngles.a3 = currentRobotJointAngles.a3 + action[2]
        nextRobotJointAngles.a4 = currentRobotJointAngles.a4 + action[3]
        nextRobotJointAngles.a5 = currentRobotJointAngles.a5 + action[4]
        nextRobotJointAngles.a6 = currentRobotJointAngles.a6 + action[5]
        #print(nextRobotJointAngles)
        return self.move(nextRobotJointAngles,train)
        
    def callbackJointAngles(self,req):
        self.robotJointAngles.a1 = req.a1
        self.robotJointAngles.a2 = req.a2
        self.robotJointAngles.a3 = req.a3
        self.robotJointAngles.a4 = req.a4
        self.robotJointAngles.a5 = req.a5
        self.robotJointAngles.a6 = req.a6

    
    def cameraCallback(self,data):
        try:
            self.image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = cv2.cvtColor(self.image_raw, cv2.COLOR_RGB2GRAY)
            self.image_resize = cv2.resize(image,(84,84)).astype(np.float32)
        except CvBridgeError as e:
            print(e)
    
    def checkScore(self):
        imageHSV = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2HSV_FULL)
        h = imageHSV[:, :, 0]
        s = imageHSV[:, :, 1]
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[((h < 20) | (h > 200)) & (s > 128)] = 255
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print("color area:")
        maxArea = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            #print(area)
            if maxArea < area:
                maxArea = area
        self.area = maxArea
        #self.area = cv2.contourArea(contours[0])
        if self.area > 5000:
            self.done = True
        if rospy.get_time() - self.startTime > 10:
            self.timeOver = True

    def reset_state(self, observation):
        # Preprocess
        obs_array = observation
        # Updates for next step
        self.last_observation = obs_array

        # Initialize State
        self.state = np.zeros((self.n_history, 84, 84), dtype=np.float32)
        self.state[0] = obs_array

    def set_state(self, observation):
        # Preproces
        obs_array = observation
        obs_processed = np.maximum(obs_array, self.last_observation)  # Take maximum from two frames

        # Updates for the next step
        self.last_observation = obs_array

        # Compose State : 4-step sequential observation
        for i in range(self.n_history - 1):
            self.state[i] = self.state[i + 1].astype(np.float32)
        self.state[self.n_history - 1] = obs_processed.astype(np.float32)

if __name__ == "__main__":
    rospy.init_node('action_client_DQN')
    print("Training start.")
    env = robotEnv()
    ra = RandomActor(env)

    n_history = env.n_history
    #print(obs_size)
    n_actions = env.actionNum
    q_func = QFunction(n_history, n_actions)
    #q_func.to_gpu(0) ## GPUを使いたい人はこのコメントを外す
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
    agent_p1 = chainerrl.agents.DoubleDQN(
        q_func, optimizer, replay_buffer, gamma, explorer,
        #replay_start_size=500, update_interval=1,
        #target_update_interval=100)
        replay_start_size=32, update_interval=1,
        target_update_interval=32*10)

    r=rospy.Rate(10)
    #学習ゲーム回数
    n_episodes = 1000
    #エピソードの繰り返し実行
    try:
        for i in range(1, n_episodes + 1):
            print("##### Episode: " + str(i) + "#####")
            env.reset()
            reward = 0
            last_state = None
            oldArea = env.area
            #while not env.done or not env.timeOver:
            while not rospy.is_shutdown():
                env.set_state(env.image_resize)
                action = agent_p1.act_and_train(env.state, reward)
                #print(action)
                #配置を実行
                if not env.move_action(action):
                    print("Problem was occured!")
                    quit()

                if env.timeOver == True:
                    if oldArea < env.area:
                        reward = 1
                    elif oldArea > env.area:
                        reward = -1
                    else:
                        reward = 0
                    
                    print("Time Over!!")
                    #学習
                    agent_p1.stop_episode_and_train(env.state, reward, True)
                    break
                
                r.sleep()

            #コンソールに進捗表示
            print("Area: ", env.area)
            if i % 10 == 0:
                print("episode:", i, " / rnd:", ra.random_count, " / area:", env.area, " / statistics:", agent_p1.get_statistics(), " / epsilon:", agent_p1.explorer.epsilon)
                ra.random_count = 0
            if i % 50 == 0:
                pass
                # 100エピソードごとにモデルを保存
                #agent_p1.save("result_joint_camera_sequence/_test_resultDQN_joint_" + str(i))
                #agent_p1.save("real_robot_test_" + str(i))

        print("Training finished.")
    except KeyboardInterrupt:
        print("Key Board Interrupt")

