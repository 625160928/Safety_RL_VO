import copy
import math

import numpy as np
import gym
import highway_env
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from halfplaneintersect import Line
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
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
        config = {
            'vehicles_count': 20,
            'simulation_frequency': 20,
            'vehicles_density': 1,
            "policy_frequency": 10,
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
        self.env.seed(11)
        self.env.configure(config)
        self.env.reset()
        self.done = False
        self.orca_policy=HighWayOrca()
        self.tau = 2
        self.dt=0.05
        self.prev = 25

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
        return action

    def get_rl_action(self,obs):
        return (0,0)

    def env_predict(self,action,obs,t):
        return obs

    def danger_action(self,action1,action2):
        return True

    def run(self):
        count = 0
        action = (0, 0)
        while not self.done:
            count += 1
            # action =env.action_space.sample()
            obs, reward, self.done, info = self.env.step(action)
            print('---------------------------------------------')
            print('run count ', count)
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
            predict_env_obs=self.env_predict(rl_action,obs,self.tau)

            #get action in predict env and check, whether action is danger
            predict_orca_action = self.get_orca_action(predict_env_obs)
            predict_rl_action = self.get_rl_action(predict_env_obs)

            if self.danger_action(predict_rl_action,predict_orca_action):
                print('danger, choose ORCA action')
                action=self.get_orca_action(obs)
            else:
                print('save, choose RL action')
                action=rl_action

            self.env.render()




def main():

   new_highway_orca=SwitchLogic()
   new_highway_orca.run()

if __name__ == '__main__':

    main()