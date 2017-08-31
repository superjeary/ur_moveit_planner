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


class QFunction(chainer.Chain):
    def __init__(self, obs_size, n_actions, n_hidden_channels=50):
        super(QFunction, self).__init__(##python2.x用
        #super().__init__(#python3.x用
            l0=L.Linear(obs_size, n_hidden_channels),
            l1=L.Linear(n_hidden_channels,n_hidden_channels),
            l2=L.Linear(n_hidden_channels, n_actions))
        
    def __call__(self, x, test=False): 
        """
        x ; 観測#ここの観測って、stateとaction両方？
        test : テストモードかどうかのフラグ
        """
        h = F.tanh(self.l0(x)) #活性化関数は自分で書くの？
        h = F.tanh(self.l1(h))
        return chainerrl.action_value.DiscreteActionValue(self.l2(h))

class RandomActor:
    def __init__(self,env):
        self.env = env
        self.random_count = 0
    def random_action_func(self):
        self.random_count += 1
        #return self.env.move_random()
        return random.randint(0,5)


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
        self.rotateUnit = 3.14/72
        self.actions = np.array([
        #[self.rotateUnit, 0.0, 0.0, 0.0, 0.0, 0.0], [-self.rotateUnit, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, self.rotateUnit, 0.0, 0.0, 0.0, 0.0], [0.0, -self.rotateUnit, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, self.rotateUnit, 0.0, 0.0, 0.0], [0.0, 0.0, -self.rotateUnit, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, self.rotateUnit, 0.0, 0.0], [0.0, 0.0, 0.0, -self.rotateUnit, 0.0, 0.0],
        #[0.0, 0.0, 0.0, 0.0, self.rotateUnit, 0.0], [0.0, 0.0, 0.0, 0.0, -self.rotateUnit, 0.0],
        #[0.0, 0.0, 0.0, 0.0, 0.0, self.rotateUnit], [0.0, 0.0, 0.0, 0.0, 0.0, -self.rotateUnit]
        ]) 

        self.robotJointAngles = moveToJointAnglesGoal()
        self.robotState = np.array([0]*6, dtype=np.float32)
        self.TCPForce = 0
        self.done = False
        self.initJointAngles = moveToJointAnglesGoal()
        self.initJointAngles.a1 = 0.0 /180*math.pi
        self.initJointAngles.a2 = -90.0 /180*math.pi
        self.initJointAngles.a3 = 90.0 /180*math.pi
        self.initJointAngles.a4 = -90.0 /180*math.pi
        self.initJointAngles.a5 = -90.0 /180*math.pi
        self.initJointAngles.a6 = 90.0 /180*math.pi

        self.initPose = moveToCartesianPoseGoal()
        self.initPose.x = 0.4869
        self.initPose.y = 0.1090
        self.initPose.z = 0.5140
        self.initPose.u = -90.0 /180*math.pi
        self.initPose.v = 0.0 /180*math.pi
        self.initPose.w = 180.0 /180*math.pi
        self.goalPose = moveToCartesianPoseGoal()

        """
        self.goalJointAngles = moveToJointAnglesGoal()
        self.goalJointAngles.a1 = -0.0171
        self.goalJointAngles.a2 = -1.512
        self.goalJointAngles.a3 = 1.505
        self.goalJointAngles.a4 = 0.007
        self.goalJointAngles.a5 = -0.012
        self.goalJointAngles.a6 = 0.012
        """

        self.goalPose = moveToCartesianPoseGoal()
        self.goalPose.x = 0.55
        self.goalPose.y = 0.365
        self.goalPose.z = 0.25
        self.goalPose.u = -90.0 /180*math.pi
        self.goalPose.v = 0.0 /180*math.pi
        self.goalPose.w = 180.0 /180*math.pi

        self.initDistance = self.calcDistance(self.initPose, self.goalPose)
        self.distance = self.calcDistance(self.initPose, self.goalPose)

        self.startTime = rospy.get_time()
        self.timeOver = False
        
        rospy.loginfo("waiting for server ...")
        _jointAnglesSub = rospy.Subscriber('/ur_moveit_planner/currentJointAngles', JointQuantity, self.callbackJointAngles)
        _poseRpySub = rospy.Subscriber('/ur_moveit_planner/currentPoseRpy', PoseRpy, self.callbackPoseRpy)
        self.action_client = actionlib.SimpleActionClient("ur_moveit_planner/moveToJointAnglesAction",moveToJointAnglesAction)
        self.action_client.wait_for_server()
        rospy.loginfo("server ON")

        self.reset()

    def reset(self):
        self.move(self.initJointAngles)
        self.TCPForce = 0
        self.done = False
        self.robotState = self.JointAngles2state(self.robotJointAngles)
        self.distance = self.calcDistance(self.robotPose.position, self.goalPose)
        self.startTime = rospy.get_time()
        self.timeOver = False
        
    def JointAngles2state(self, JointAngles):
        state = np.array([0]*6, dtype=np.float32)
        state[0]=JointAngles.a1
        state[1]=JointAngles.a2
        state[2]=JointAngles.a3
        state[3]=JointAngles.a4
        state[4]=JointAngles.a5
        state[5]=JointAngles.a6     

        return state

    def move(self, targetJointAngles):
        self.action_client.send_goal(targetJointAngles)
        finished = self.action_client.wait_for_result(rospy.Duration(5.0))
        if finished:
            result = self.action_client.get_state();
            #print result
            self.robotState = self.JointAngles2state(self.robotJointAngles)
            self.check_goal()
            #self.done = True
            return True
        else:
            print "time out!!"
            self.robotState = self.JointAngles2state(self.robotJointAngles)
            return False

    def move_action(self, act):
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
        return self.move(nextRobotJointAngles)

    def move_random(self):
        r = random.randint(0,5)
        action = self.actions[r] 
        nextRobotJointAngles = moveToJointAnlgesGoal()
        currentRobotJointAngles = copy.deepcopy(self.robotJointAngles)
        
        nextRobotJointAngles.a1 = currentRobotJointAngles.a1 + action[0]
        nextRobotJointAngles.a2 = currentRobotJointAngles.a2 + action[1]
        nextRobotJointAngles.a3 = currentRobotJointAngles.a3 + action[2]
        nextRobotJointAngles.a4 = currentRobotJointAngles.a4 + action[3]
        nextRobotJointAngles.a5 = currentRobotJointAngles.a5 + action[4]
        nextRobotJointAngles.a6 = currentRobotJointAngles.a6 + action[5]

        return self.move(nextRobotJointAngles)
        
    def callbackJointAngles(self,req):
        self.robotJointAngles.a1 = req.a1
        self.robotJointAngles.a2 = req.a2
        self.robotJointAngles.a3 = req.a3
        self.robotJointAngles.a4 = req.a4
        self.robotJointAngles.a5 = req.a5
        self.robotJointAngles.a6 = req.a6

    def callbackPoseRpy(self, req):
        self.robotPose = req

    def check_goal(self):
        currentPose = copy.deepcopy(self.robotPose)
        self.distance = self.calcDistance(currentPose.position, self.goalPose)

        if self.distance < 0.1:
            self.done = True
        #print(rospy.get_time() - self.startTime)
        if rospy.get_time() - self.startTime > 10:
            self.timeOver = True
    
    def calcDistance(self,a,b):
        diffX = a.x - b.x
        diffY = a.y - b.y
        diffZ = a.z - b.z
        d = math.sqrt(diffX*diffX+diffY*diffY+diffZ*diffZ)
        #print("current disntance" + str(d)) 
        return d

if __name__ == "__main__":
    rospy.init_node('action_client_DQN')
    print("Training start.")
    env = robotEnv()
    ra = RandomActor(env)

    obs_size = 6
    n_actions = 6
    q_func = QFunction(obs_size, n_actions)
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
    agent_p1 = chainerrl.agents.DoubleDQN(
        q_func, optimizer, replay_buffer, gamma, explorer,
        replay_start_size=500, update_interval=1,
        target_update_interval=100)

    #学習ゲーム回数
    n_episodes = 100
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
            #print(action)
            #配置を実行
            if not env.move_action(action):
                "Problem is occured!"
                quit()
            #配置の結果、終了時には報酬とカウンタに値をセットして学習
            if env.done == True:
                reward = 5
                
                #エピソードを終了して学習
                agents.stop_episode_and_train(env.robotState.copy(), reward, True)
                break
            elif env.timeOver == True:
                print("Time Over!!")
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
        print("distance: ", env.distance)
        if i % 10 == 0:
            print("episode:", i, " / rnd:", ra.random_count, " / distance:", env.distance, " / statistics:", agent_p1.get_statistics(), " / epsilon:", agent_p1.explorer.epsilon)
            ra.random_count = 0
        if i % 50 == 0:
            # 10000エピソードごとにモデルを保存
            agent_p1.save("resultDQN_pose_" + str(i))

    print("Training finished.")