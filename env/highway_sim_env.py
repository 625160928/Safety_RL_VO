

import math

import numpy as np
import gym
import highway_env
import merge_env

class HighwaySimulation():
    def __init__(self,env_name,config):
        #参数
        self.parm_config=config
        self.policy_frequency = config['policy_frequency']
        self.dt = config['dt']
        self.tau = config['tau']
        self.prev =config['prev']
        self.sim_env_config = {
            "lanes_count": config['lanes_count'],
            "ego_spacing": 1,
            'vehicles_count': config['vehicles_count'],
            'simulation_frequency': 1 / self.dt,  # 20
            'vehicles_density': config['vehicles_density'],
            "policy_frequency": self.policy_frequency,  # 10
            "duration": config['duration'],
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 6,
                "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
                "absolute": True,
                "normalize": False,
                "order": "sorted"
            },
            "action": {
                "type": "ContinuousAction",
                "STEERING_RANGE": (-config['car_steer_limit'], config['car_steer_limit'])
            }
        }

        #仿真环境
        self.env = gym.make(env_name)
        self.done = False
        self.env.seed(config['seed'])
        self.predict_time = 2
        self.pose = []
        self.pred = []
        self.reinit()
        self.road=self.env.road

    def step(self,action):
        obs, reward, self.done, info =self.env.step(action)
        return obs, reward, self.done, info

    def reinit(self):
        self.env.configure(self.sim_env_config)
        self.env.reset()
        controller_vehicle=self.env.vehicle
        controller_vehicle.position=controller_vehicle.position+np.array([-9,0])

    def render(self):
        self.env.render()


    def env_predict(self, action, obs):
        update_predit_time = self.predict_time * (1 / self.dt) / self.policy_frequency
        pre_obs_arr = []
        # print(time+self.predict_time)
        for vehicle in self.env.road.vehicles:
            for obj in obs:
                if obj[1] == vehicle.position[0] and obj[2] == vehicle.position[1]:
                    pre_obj = vehicle.predict_trajectory_constant_speed([update_predit_time])
                    # print(pre_obj[0][0][0],pre_obj[1])
                    pre_obs = [1, pre_obj[0][0][0], pre_obj[0][0][1], obj[3], obj[4], math.cos(pre_obj[1][0]),
                               math.sin(pre_obj[1][0])]
                    # print('predict ',pre_obs)
                    pre_obs_arr.append(pre_obs)
        # print(len(obs),len(pre_obs_arr),len(obs[0]),len(pre_obs_arr[0]))

        return pre_obs_arr

    def get_cloest_distance(self,obs):
        my_x=obs[0][1]
        my_y=obs[0][2]
        min_dis=999999999999
        for i in range(1,len(obs)):
            now_x=obs[i][1]
            now_y=obs[i][2]
            dis=math.hypot(now_x-my_x,now_y-my_y)
            if dis<min_dis:
                min_dis=dis

        return min_dis