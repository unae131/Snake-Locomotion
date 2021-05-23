from os import setegid
from numpy.core.defchararray import endswith
import pybullet as p
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import random
from gym.envs.classic_control.SnakeModel import *

EPSILON = 0.00000001

class SnakeEnv(gym.Env):

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        # init STATE, ACTION SPACE
        self.threshold_velocity = 0.05
        self.state = None
        self.goal = []
        self.snake = Snake(20)
        self.forces = None

        threshold = 600
        threshold_low = np.concatenate((np.full(self.snake.nodeNum*6,-threshold), [-20.,-20.])) # coordinates + velocities
        threshold_high = np.concatenate((np.full(self.snake.nodeNum*6, threshold), [20., 20.]))

        # # method1
        act_low = []
        act_high = []
        act_x_offset, act_y_offset, act_z_offset = np.pi/6, np.pi/2, np.pi/6
        force_low = 100
        force_high = 400

        # for i in range(self.snake.jointNum):
        #     act_low += [-act_y_offset] # [-act_x_offset, -act_y_offset, -act_z_offset]
        #     act_high += [act_y_offset] # [act_x_offset, act_y_offset, act_z_offset]
        
        # for i in range(self.snake.jointNum):
        #     act_low += [force_low, force_low, force_low]
        #     act_high += [force_high, force_high, force_high]
        
        # method2
        steering = 0.2
        # force = 300.
        # dt = 1./30.
        # wavePeriod = 3
        # waveLength = 10.
        # waveAmplitude = 2

        # act_low = np.array([-steering, force-100, 1./480., 1, 1., 0.1])
        # act_high = np.array([steering, force, dt, wavePeriod, waveLength, waveAmplitude])
        act_low = np.array([-steering])#, 100, 0.5, 0.8])
        act_high = np.array([steering])#, force, wavePeriod, waveAmplitude])

        self.action_space = spaces.Box(low=np.array(act_low), high=np.array(act_high))
        self.observation_space = spaces.Box(low= threshold_low, high= threshold_high)

        self.tmp = None
        self.tmp1 = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # action = np.array(action) * 0.01
        # past_state = self.state
        past_midWorldPos = self.snake.midWorldPos
        # past_tailToHead = self.snake.headWorldPos - self.snake.tailWorldPos
        # past_tailToHead /= np.linalg.norm(past_tailToHead)
        # past_headToGoal = self.goal - self.snake.headWorldPos
        # past_headToGoal /= np.linalg.norm(past_headToGoal)
        # past_dot = np.dot(past_headToGoal, past_tailToHead)
        
        # for debug
        past_head = self.snake.headWorldPos

        # self.snake.moveSin1(steering = 0.0, force = 100, dt = 1./20.,
        #                     wavePeriod = 0.5, waveAmplitude = 1)
        # self.snake.moveSin1(steering = action[0], force = action[1], dt = 1./30.,
        #                     wavePeriod = action[2], waveAmplitude = action[3])
        self.snake.moveSin(steering = action[0],#action[0],
                            forces = self.forces, dt = 1./30.,
                            wavePeriod = 1.5, waveLength = 4, waveAmplitude = 0.8)
        # self.snake.move(action[:self.snake.nodeNum], action[self.snake.nodeNum:])

        newState = self.snake.linksLocalPositionAndVelocity # 이건 전부 Local(mid Node를 포함해서!)
        localGoal = self.snake.getLocalCoordinate(self.goal - self.snake.midWorldPos) # pos으로 계산
        newState = np.concatenate((newState, localGoal[:2]))

        rewards = []
        weights = []

        # reward_1 : head local dir 벡터와 local head to local goal벡터 사이 각을 최소화
        # headDirVec = self.snake.headLocalAheadDir
        # headToGoalVec = np.array(localGoal) - np.array(self.state[:3])
        # rewards.append(np.pi - self.getTheta(headDirVec, headToGoalVec))
        
        headWorldDir = np.reshape(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(self.snake.boneId)[1]), (3,3))@[0.,0.,-1.]
        worldHeadToGoal = self.goal-self.snake.headWorldPos
        headWorldDir /= np.linalg.norm(headWorldDir)
        worldHeadToGoal /= np.linalg.norm(worldHeadToGoal)
        rewards.append(np.dot(headWorldDir, worldHeadToGoal)**3)
        weights.append(10.)

        # # reward_2 : goal local벡터와 몸통의 y축(tail to head) 각도를 최소화
        # rewards.append(np.pi - self.getTheta(localGoal, np.array([0.,1.,0.])))
        headToGoal = self.goal - self.snake.headWorldPos
        headToGoal /= np.linalg.norm(headToGoal)
        tailToHead = self.snake.headWorldPos - self.snake.tailWorldPos#(self.snake.midWorldTrans4M @ np.array([0.,1.,0.,0.]))[:3]
        # tailToHead /= np.linalg.norm(tailToHead)
        new_dot = np.dot(headToGoal, tailToHead)
        rewards.append(new_dot) #if new_dot > past_dot else 0)
        # # print(np.pi - rewards[-1])
        weights.append(1.)

        
        # # reward_3 : 몸통 y 축과 내적 최대
        # # rewards.append(np.dot(self.snake.totalLocalVel, np.array([0.,1.,0.])))

        # # reward_4 : goal까지의 거리를 최소화
        # rewards.append(1 / (np.linalg.norm(self.state[:3] - localGoal) + 1e-6))
        rewards.append(1 / (np.linalg.norm(self.snake.headWorldPos - self.goal) + 1e-6))
        weights.append(100.)

        # # reward_5 : torque minimization / 1e+6 단위
        # rewards.append(1 / (self.snake.getJointsTorqueSum() + 1e-6))
        # weights.append(1000.)

        # # reward_6 : acceleration 최소 / 100단위
        # rewards.append(1 / (np.sum(np.abs(np.array(self.state[3*self.snake.nodeNum:]) - np.array(past_state[3*self.snake.nodeNum:])))+1e-6))
        # weights.append(1.)

        # lastVeloSum = np.reshape(past_state[3*self.snake.nodeNum:], (self.snake.nodeNum, 3))
        # lastVeloSum.sum(axis = 0)
        # reward_6 = 1 / (np.sum(np.abs(np.array(self.snake.totalLocalVel) - lastVeloSum))+1e-6)
        # weights.append(1.)

        # # reward_7 : 위 아래로 관절 움직이는 것은 최소
        # # actionSum = np.reshape(action, (self.snake.jointNum, 3)).sum(axis = 0)
        # # reward_7 = 1 / (actionSum[0] + actionSum[2])
        # rewards.append(-action[0]) # steering
        # weights.append(0.)

        # # reward_8 : 몸이 최대한 펴지도록
        # rewards.append(np.linalg.norm(self.snake.headWorldPos - self.snake.tailWorldPos))
        # weights.append(10.)

        # reward : 머리 방향과 steering값 비교
        # headDir = headWorldDir #p.self.snake.headLocalAheadDir
        # headToGoal = worldHeadToGoal#localGoal - self.state[:3]
        # sin = np.linalg.norm(np.cross(headDir, headToGoal)) / (np.linalg.norm(headDir)*np.linalg.norm(headToGoal))
        # if sin > 0 and action[0] > 0:
        #     rewards.append(1.)
        # elif sin > 0:
        #     rewards.append(-10.)
        # elif sin < 0 and action[0] < 0:
        #     rewards.append(1.)
        # elif sin < 0:
        #     rewards.append(-10.)
        # elif sin == 0 and action[0] == 0:
        #     rewards.append(2.)
        # elif sin == 0:
        #     rewards.append(-5.)
        # weights.append(10.)

        # reward : 중앙 노드의 이동방향과 중앙에서 골까지 내적
        # midToGoalUnitVec = self.goal - self.snake.midWorldPos
        # midToGoalUnitVec /= np.linalg.norm(midToGoalUnitVec)
        # rewards.append(np.dot(midToGoalUnitVec,(self.snake.midWorldPos - past_midWorldPos) ))
        # weights.append(500.)

        # print(rewards, action[0])
        # print(rewards)

        # print(rewards, weights)
        reward = .1*(np.dot(weights, rewards))

        done = 1
        # if np.dot((self.snake.headWorldPos - past_head), worldHeadToGoal) < 0:
        #     done = 1
        # else:
        #     done = 0

        # debug
        if self.tmp is not None:
            p.removeUserDebugItem(self.tmp)
            p.removeUserDebugItem(self.tmp1)
            # p.removeUserDebugItem(self.tmp2)
        
        worldheadToGoal = (self.snake.midWorldTrans4M @ np.concatenate((localGoal - self.state[:3], [0.])))[:3]
        worldMidToGoal = (self.snake.midWorldTrans4M @ np.concatenate((localGoal, [1.])))[:3]
        self.tmp = p.addUserDebugLine(self.snake.headWorldPos, self.snake.headWorldPos + worldheadToGoal, lineColorRGB = [1,0,0]) 
        self.tmp1 = p.addUserDebugLine(self.snake.midWorldPos, self.snake.midWorldPos + worldMidToGoal, lineColorRGB = [0,1,1])
        # self.tmp1 = p.addUserDebugLine(self.snake.midWorldTrans4M @ np.concatenate((localGoal, [1.])))[:3]
        # self.tmp2 = p.addUserDebugLine(self.snake.headWorldPos, self.goal, lineColorRGB = [0,0,1]) 
        # self.tmp2 = p.addUserDebugLine(self.snake.headWorldPos, self.snake.headWorldPos + (self.snake.midWorldTrans4M @ np.concatenate((localGoal, [1.])))[:3], lineColorRGB = [0,0,1])
        p.addUserDebugLine(self.snake.headWorldPos, past_head, lineColorRGB = [1, 0, 1])
        p.addUserDebugLine(self.snake.midWorldPos, past_midWorldPos, lineColorRGB = [0, 1, 0])

        # goal 재설정
        prob = 0.00
        if self.isInGoal() or random.random() < prob:
            self.goalBox.remove()
            self.goal, self.goalBox = self.resetGoal()

        return self.state, reward, done, {}

    def reset(self):
        self.snake.init(soft=True)

        self.forces = np.full((self.snake.jointNum, 3), 200, dtype=np.float32)
        # initAction = np.reshape([[0.,random.uniform(-np.pi/2,np.pi/2),0.] for i in range(self.snake.jointNum)],(self.snake.jointNum*3,))
        # self.snake.move(initAction, self.forces)

        self.goal, self.goalBox = self.resetGoal(base = self.snake.midWorldPos)
        self.state = np.concatenate((self.snake.linksLocalPositionAndVelocity, (self.goal - self.snake.midWorldPos)[:2]))
        
        return self.state # must be np.array

    def isInGoal(self):
        for i in range(3):
            if self.goal[i] - 1 > self.snake.headWorldPos[i] or self.goal[i] + 1 < self.snake.headWorldPos[i]:
                return False

        return True

    def resetGoal(self, base = [0.,0.,0.], offset = 20):
        # worldGoal = [random.uniform(base[0]-offset,base[0]+offset),random.uniform(base[1]-offset, base[1]+offset), 0.] # 속도(방향 + 속도 고려하여) 벡터로 표현, 로컬 좌표계 기준
        worldGoal = [6.,-1.,0]
        goalBox = BoxNode(worldGoal, 0.5)
        # print(worldGoal)

        return worldGoal, goalBox

    def getTheta(self, vecA, vecB):
        # 제 2 cos
        vecC = vecB - vecA

        norm_A = np.linalg.norm(vecA)
        norm_B = np.linalg.norm(vecB)
        norm_C = np.linalg.norm(vecC)

        # if norm_A == 0 or norm_B == 0:
        #     return 0
        
        theta = np.arccos((norm_A ** 2 + norm_B ** 2 - norm_C ** 2) / (2 * norm_A * norm_B))
        return theta
