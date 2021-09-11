import copy
import math

import numpy
import numpy as np
import gym
import highway_env
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp, Line

import pyorca
from controller import pid_lateral_controller_angle
# from controller import pid_longitudinal_controller
from highway import HighWayOrca

class SwitchLogic():
    def __init__(self):
        self.policy1=0
        self.car_steer_limit = math.pi / 3
        self.env = gym.make("highway-v0")
        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)
        self.policy_frequency=10
        self.dt=0.05
        self.done = False
        self.orca_policy=HighWayOrca()
        self.tau = 2
        self.prev = 25
        self.env.seed(11)
        self.switch_danger_theta=math.pi/6
        self.predict_time=1
        self.pose=[]
        self.pred=[]
        config = {
            'vehicles_count': 30,
            'simulation_frequency': 1/self.dt,
            'vehicles_density': 1,
            "policy_frequency": self.policy_frequency,
            "duration": 1000,
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 15,
                "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
                "features_range": {
                    "x": [-100, 100],
                    "y": [-100, 100],
                    "vx": [-20, 20],
                    "vy": [-20, 20]
                },
                "absolute": True,
                "normalize": False,
                "order": "sorted"
            },
            "action": {
                "type": "ContinuousAction"
            }
        }
        self.env.configure(config)
        self.env.reset()

    #et orca action from orca
    def get_orca_action(self,obs):
        agents = []
        for obj in obs:
            pd = False
            for i in obj:
                if i != 0:
                    pd = True
            if pd:
                agents.append(self.orca_policy.get_agent_from_obs(obj))
                # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )


        # 设置目标速度，换道决策
        agents[0].pref_velocity = self.orca_policy.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                  agents[0].position[0] + 80, self.orca_policy.lane * self.orca_policy.lane_length,
                                                  self.orca_policy.prev)
        # print('pose ',agents[0].position,' theta ',theta*180/math.pi,math.sin(theta))

        # 计算orca避障速度
        new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt,
                                  limit=[-2 + agents[0].radius / 2 + self.orca_policy.edge_remain,
                                         14 - agents[0].radius / 2 - self.orca_policy.edge_remain])

        # 将速度转换为动作指令
        action = self.orca_policy.change_vxvy_to_action(agents[0], new_vels)
        return action,new_vels

    def get_rl_action(self,obs):
        return (0,0)

    def env_predict(self,action,obs,t,time):
        update_predit_time=self.predict_time*(1/self.dt)/self.policy_frequency
        pre_obs_arr=[]
        # print(time+self.predict_time)
        for vehicle in self.env.road.vehicles:
            for obj in obs:
                if obj[1]==vehicle.position[0] and obj[2]==vehicle.position[1]:
                    pre_obj=vehicle.predict_trajectory_constant_speed([update_predit_time])
                    # print(pre_obj[0][0][0],pre_obj[1])
                    pre_obs=[1,pre_obj[0][0][0],pre_obj[0][0][1],obj[3],obj[4],math.cos(pre_obj[1][0]),math.sin(pre_obj[1][0])]
                    # print('predict ',pre_obs)
                    pre_obs_arr.append(pre_obs)
        # print(len(obs),len(pre_obs_arr),len(obs[0]),len(pre_obs_arr[0]))

        return pre_obs_arr


    def danger_action(self,env_obs,orca_speed):
        position_sin_theta=env_obs[0][6]
        position_cos_theta=env_obs[0][5]

        a=math.hypot(position_cos_theta,position_sin_theta)
        b=math.hypot(orca_speed[0],orca_speed[1])
        c=math.hypot(orca_speed[0]-position_cos_theta,orca_speed[1]-position_sin_theta)

        derta_cos_theta=(a*a+b*b-c*c)/(2*a*b)
        derta_cos_theta=numpy.clip(derta_cos_theta,-1,1)
        if abs(math.acos(derta_cos_theta))>self.switch_danger_theta:
            return True
        return True
        # return False

    def run(self):
        count = 0
        action = (0, 0)
        while not self.done:
            print('---------------------------------------------')
            count += 1
            print('run count ', count,"  time = ",count*self.dt)
            print('final action is ',action)
            # action =env.action_space.sample()
            obs, reward, self.done, info = self.env.step(action)
            self.pose.append([count*self.dt,[obs[0][1],obs[0][2]]])
            print("now pose is ",obs[0][1],obs[0][2],' speed is ',[obs[0][3],obs[0][4]])
            # print('action ',action, type(action))
            # print('obs ',obs)

            if self.done:
                if info["crashed"] == True:
                    print("crash")
                else:
                    print("done")
                break

            #get rl action in current env
            rl_action=self.get_rl_action(obs)

            #predict env at time=self.tau later
            predict_env_obs=self.env_predict(rl_action,obs,self.tau,count*self.dt)

            #get action in predict env and check, whether action is danger
            predict_orca_action,pre_vel_speed = self.get_orca_action(predict_env_obs)

            if self.danger_action(predict_env_obs,pre_vel_speed):
                print('danger, choose ORCA action')
                action,vel_speed=self.get_orca_action(obs)
            else:
                print('save, choose RL action')
                action=rl_action

            self.env.render()
        print('----------------------------------------------------------')
        diff=int( self.predict_time/self.dt)
        print(diff)
        for i in range(len(self.pose)):
            if i-diff<0:
                continue
            else:
                print(self.pose[i][1],' --- ',self.pred[i-diff][1])



def main():

   new_highway_orca=SwitchLogic()
   new_highway_orca.run()

if __name__ == '__main__':

    main()