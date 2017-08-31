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

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from ur_moveit_planner.msg import PoseRpy


#Q関数
class QFunction(chainer.Chain):
    def __init__(self, obs_size, n_actions, n_hidden_channels=81):
        super(QFunction, self).__init__(
            l0=L.Linear(obs_size, n_hidden_channels),
            l1=L.Linear(n_hidden_channels, n_hidden_channels),
            l2=L.Linear(n_hidden_channels, n_hidden_channels),
            l3=L.Linear(n_hidden_channels, n_actions))
    def __call__(self, x, test=False):
        #-1を扱うのでleaky_reluとした
        h = F.leaky_relu(self.l0(x))
        h = F.leaky_relu(self.l1(h))
        h = F.leaky_relu(self.l2(h))
        return chainerrl.action_value.DiscreteActionValue(self.l3(h))

class RandomActor:
    def __init__(self,env):
        self.env = env
        self.random_count = 0
    def random_action_func(self):
        self.random_count += 1
        #return self.env.move_random()
        return random.randint(0,3)

# Define Agent And Environment
class robotAgent:
    def __init__(self):
        pass
  
    def loadAgent(self,fname):
        # optimizer, qfuncをagentに入れ込む
        self.q_func = QFunction()
        self.optimizer = chainer.optimizers.Adam(eps=1e-2)
        self.optimizer.setup(self.q_func)

        # この辺、公式チュートリアルのコピペ。問題によって調整...知らんがな
        # https://github.com/chainer/chainerrl/blob/master/examples/quickstart/quickstart.ipynb
        self.gamma = 0.95
        self.explorer = chainerrl.explorers.ConstantEpsilonGreedy(
                epsilon=0.05, random_action_func = ra.random_action_func)
        self.replay_buffer = chainerrl.replay_buffer.ReplayBuffer(capacity=10 ** 6)
        self.phi = lambda x: x.astype(np.float32, copy=False)
        
        self.agent = chainerrl.agents.DoubleDQN(
                self.q_func, self.optimizer, self.replay_buffer, self.gamma, self.explorer,
                replay_start_size=500,  phi=self.phi)
        
        if fname != "":
            print("load agent from : " + fname)
            self.agent.load(fname)    

class robotEnv:
    def __init__(self):
        self.coordinateUnit = 0.01
        self.rotateUnit = 1.0
        """
        self.actions = np.array([
        [self.coordinateUnit, 0.0, 0.0, 0.0], [-self.coordinateUnit, 0.0, 0.0, 0.0],
        [0.0, self.coordinateUnit, 0.0, 0.0], [0.0, -self.coordinateUnit, 0.0, 0.0],
        [0.0, 0.0, self.coordinateUnit, 0.0], [0.0, 0.0, -self.coordinateUnit, 0.0],
        [0.0, 0.0, 0.0, self.rotateUnit], [0.0, 0.0, 0.0, -self.rotateUnit]]) 
        """
        self.actions = np.array([
        [self.coordinateUnit, 0.0, 0.0, 0.0], [-self.coordinateUnit, 0.0, 0.0, 0.0],
        [0.0, 0.0, self.coordinateUnit, 0.0], [0.0, 0.0, -self.coordinateUnit, 0.0]]) 

        self.robotPose = PoseRpy()
        self.robotState = np.array([0]*6, dtype=np.float32)
        self.TCPForce = 0
        self.done = False
        self.initPose = moveToCartesianPoseGoal()
        """
        self.initPose.x = 0.567
        self.initPose.y = 0.365
        self.initPose.z = 0.416
        self.initPose.u = 3.14
        self.initPose.v = 0
        self.initPose.w = 1.57
        """
        self.initPose.x = 0.81
        self.initPose.y = 0.19
        self.initPose.z = -0.0054
        self.initPose.u = 3.14
        self.initPose.v = 0
        self.initPose.w = 1.57
        self.goalPose = moveToCartesianPoseGoal()

        """
        self.goalPose.x = 0.567
        self.goalPose.y = 0.365
        self.goalPose.z = 0.6
        self.goalPose.u = 3.14
        self.goalPose.v = 0
        self.goalPose.w = 1.57
        """
        self.goalPose.x = 0.4
        self.goalPose.y = 0.19
        self.goalPose.z = 0.4
        self.goalPose.u = -3.14
        self.goalPose.v = 0
        self.goalPose.w = 1.57

        self.initDistance = self.calcDistance(self.initPose, self.goalPose)
        self.distance = self.calcDistance(self.initPose, self.goalPose)

        self.startTime = rospy.get_time()
        self.timeOver = False
        
        rospy.loginfo("waiting for server ...")
        sub = rospy.Subscriber('/ur_moveit_planner/currentPoseRpy', PoseRpy, self.callback)
        self.action_client = actionlib.SimpleActionClient("ur_moveit_planner/moveToCartesianPoseAction",moveToCartesianPoseAction)
        self.action_client.wait_for_server()
        rospy.loginfo("server ON")

        self.reset()

    def reset(self):
        self.move(self.initPose)
        self.TCPForce = 0
        self.done = False
        self.robotState = self.pose2state(self.robotPose)
        self.distance = self.calcDistance(self.initPose, self.goalPose)
        self.startTime = rospy.get_time()
        self.timeOver = False
        
    def pose2state(self, pose):
        state = np.array([0]*6, dtype=np.float32)
        state[0]=pose.position.x
        state[1]=pose.position.y
        state[2]=pose.position.z
        state[3]=pose.rpy.u
        state[4]=pose.rpy.v
        state[5]=pose.rpy.w     

        return state

    def move(self, targetPose):
        self.action_client.send_goal(targetPose)
        finished = self.action_client.wait_for_result(rospy.Duration(5.0))
        if finished:
            result = self.action_client.get_state();
            #print result
            self.robotState = self.pose2state(self.robotPose)
            self.check_goal()
            return True
        else:
            print "time out!!"
            self.robotState = self.pose2state(self.robotPose)
            return False

    def move_action(self, act):
        action = self.actions[act]        
        #print act, action
        nextRobotPose = moveToCartesianPoseGoal()
        currentRobotPose = copy.deepcopy(self.robotPose )
        
        nextRobotPose.x = currentRobotPose.position.x + action[0]
        nextRobotPose.y = currentRobotPose.position.y + action[1]
        nextRobotPose.z = currentRobotPose.position.z + action[2]
        
        nextRobotPose.u = currentRobotPose.rpy.u
        nextRobotPose.v = currentRobotPose.rpy.v
        nextRobotPose.w = currentRobotPose.rpy.w

        return self.move(nextRobotPose)


    def move_random(self):
        r = random.randint(0,6)
        action = self.actions[r] 
        nextRobotPose = moveToCartesianPoseGoal()
        currentRobotPose = copy.deepcopy(self.robotPose )
        
        nextRobotPose.x = currentRobotPose.position.x + action[0]
        nextRobotPose.y = currentRobotPose.position.y + action[1]
        nextRobotPose.z = currentRobotPose.position.z + action[2]

        nextRobotPose.u = currentRobotPose.rpy.u
        nextRobotPose.v = currentRobotPose.rpy.v
        nextRobotPose.w = currentRobotPose.rpy.w

        return self.move(nextRobotPose)
        
    def callback(self, req):
        self.robotPose = req

    def check_goal(self):
        currentPose = copy.deepcopy(self.robotPose)
        #diffX = currentPose.position.x - self.goalPose.x
        #diffY = currentPose.position.y - self.goalPose.y
        #diffZ = currentPose.position.z - self.goalPose.z
        #self.distance = math.sqrt(diffX*diffX+diffY*diffY+diffZ*diffZ)
        self.distance = self.calcDistance(currentPose.position, self.goalPose)

        if self.distance < 0.01:
            self.done = True
        #print(rospy.get_time() - self.startTime)
        if rospy.get_time() - self.startTime > 30:
            self.timeOver = True
    
    def calcDistance(self,a,b):
        diffX = a.x - b.x
        diffY = a.y - b.y
        diffZ = a.z - b.z
        d = math.sqrt(diffX*diffX+diffY*diffY+diffZ*diffZ)
        #print("current disntance: " + str(d)) 
        return d
if __name__ == "__main__":
    rospy.init_node('action_client_DQN')
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
        start_epsilon=1.0, end_epsilon=0.3, decay_steps=20000, random_action_func=ra.random_action_func)
    # Experience ReplayというDQNで用いる学習手法で使うバッファ
    replay_buffer = chainerrl.replay_buffer.ReplayBuffer(capacity=10 ** 6)
    # Agentの生成（replay_buffer等を共有する2つ）
    agent_p1 = chainerrl.agents.DoubleDQN(
        q_func, optimizer, replay_buffer, gamma, explorer,
        replay_start_size=500, update_interval=1,
        target_update_interval=100)

    #学習ゲーム回数
    n_episodes = 10000
    #エピソードの繰り返し実行
    for i in range(1, n_episodes + 1):
        print("##### Episode: " + str(i) + "#####")
        env.reset()
        reward = 0
        agents = agent_p1
        last_state = None
        #while not env.done or not env.timeOver:
        while not rospy.is_shutdown():
            old_dist = env.distance
            action = agents.act_and_train(env.robotState.copy(), reward)
            #配置を実行
            env.move_action(action)
            #配置の結果、終了時には報酬とカウンタに値をセットして学習
            if env.done == True:
                reward = 5
                #エピソードを終了して学習
                agents.stop_episode_and_train(env.robotState.copy(), reward, True)
                break
            elif env.timeOver == True:
                #print("Time Over!!")
                if env.initDistance > env.distance:
                    reward = 1
                elif env.initDistance < env.distance:
                    reward = -1
                else:
                    reward = 0
                #エピソードを終了して学習
                agents.stop_episode_and_train(env.robotState.copy(), reward, True)
                break
            else:
                #学習用にターン最後の状態を退避
                last_state = env.robotState.copy()

        #コンソールに進捗表示
        if i % 100 == 0:
            print("episode:", i, " / rnd:", ra.random_count, " / distance:", env.distance, " / statistics:", agent_p1.get_statistics(), " / epsilon:", agent_p1.explorer.epsilon)
            ra.random_count = 0
        if i % 2000 == 0:
            # 10000エピソードごとにモデルを保存
            agent_p1.save("resultDQN_pose_" + str(i))

    print("Training finished.")