import copy
import math

import numpy
import numpy as np
import gym
import highway_env
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from car_orca.pyorca_master.pyorca import Agent, get_avoidance_velocity, orca, normalized, perp, Line

import car_orca.pyorca_master.pyorca
from controller import pid_lateral_controller_angle
# from controller import pid_longitudinal_controller
from car_orca.pyorca_master.highway import HighWayOrca

import torch as th
from stable_baselines3 import PPO
from torch.distributions import Categorical
import torch
import torch.nn as nn
from torch.nn import functional as F


class SwitchLogic():
    def __init__(self,seed=0):
        self.policy1=0
        self.car_steer_limit = math.pi / 3
        self.env = gym.make("highway-v0")
        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)
        self.policy_frequency=10
        self.dt=0.05
        self.done = False
        self.orca_policy=HighWayOrca()
        self.tau = 2
        self.prev = 20
        self.orca_policy.prev=self.prev
        self.orca_policy.tau=self.tau
        self.env.seed(seed)
        self.switch_danger_theta=math.pi/6
        self.switch_danger_dis=3.5
        self.predict_time=2
        self.pose=[]
        self.pred=[]
        self.config = {
            "lanes_count":3,
            "ego_spacing":1,
            'vehicles_count': 15,
            'simulation_frequency': 1/self.dt,#20
            'vehicles_density': 1.5,
            "policy_frequency": self.policy_frequency,#10
            "duration": 200,
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
                "STEERING_RANGE" : (-np.pi/3, np.pi/3)
            }
        }
        self.reinit()

    def reinit(self):
        self.env.configure(self.config)
        self.env.reset()
        controller_vehicle=self.env.vehicle
        controller_vehicle.position=controller_vehicle.position+np.array([-9,0])


    #et car_orca action from car_orca
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

    def get_rl_action(self,model,obs):
        # action, _ = model.predict(obs)
        # print(action)
        return (0,0)
        # return action

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
        # a=math.hypot(position_cos_theta,position_sin_theta)
        # b=math.hypot(orca_speed[0],orca_speed[1])
        # c=math.hypot(orca_speed[0]-position_cos_theta,orca_speed[1]-position_sin_theta)

        now_speed_x=env_obs[0][3]
        now_speed_y=env_obs[0][4]
        derta_speed_x=orca_speed[0]-now_speed_x
        derta_speed_y=orca_speed[1]-now_speed_y

        #使用预测后当前速度与预测后orca速度的速度差的角度作为危险判断
        a=math.hypot(now_speed_x,now_speed_y)
        b=math.hypot(derta_speed_x,derta_speed_y)
        c=math.hypot(derta_speed_x-now_speed_x,derta_speed_y-now_speed_y)



        if a==0 :
            a=0.000000000001
        if b==0:
            b=0.00000000000001
        derta_cos_theta=(a*a+b*b-c*c)/(2*a*b)
        derta_cos_theta=numpy.clip(derta_cos_theta,-1,1)



        # print('predict ',env_obs[0])
        # print('predict now speed ',now_speed_x,now_speed_y,' orca ',orca_speed,' derta_speed ',derta_speed_x,derta_speed_y)
        # print('predict position theta ',math.atan2(position_sin_theta,position_cos_theta),' orca v theta ',math.atan2(orca_speed[1],orca_speed[0]))

        # print(' danger angle ',abs(math.acos(derta_cos_theta))*180/math.pi,' limit  ',self.switch_danger_theta*180/math.pi)
        # if abs(math.acos(derta_cos_theta))>self.switch_danger_theta:
        #     return True
        # print('danger dis ',math.hypot(derta_speed_x,derta_speed_y),self.switch_danger_dis)
        if math.hypot(derta_speed_x,derta_speed_y)>self.switch_danger_dis:
            return True

        # return True
        return False

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

    def run(self):
        in_target_lan_count=0
        crash=False
        speed_total=0
        min_dis_total=0
        min_dis=99999999999999
        leagle=True

        model=0
        # model = PPO.load("./ppo-highway8")

        count = 0
        action = (0, 0)
        rl_count=0
        while not self.done:
            # print('----------------------------------------------------------')
            count += 1
            # print('---------------------------------------------')
            # print('run count ', count,"  time = ",count*self.dt)
            # print('final action is ',action)

            obs, reward, self.done, info = self.env.step(action)
            self.pose.append([count*self.dt,[obs[0][1],obs[0][2]]])
            # print("now pose is ",obs[0][1],obs[0][2],' speed is ',[obs[0][3],obs[0][4]])

            #collect speed info
            speed_total+=obs[0][3]

            tmp_min_dis=self.get_cloest_distance(obs)
            if tmp_min_dis<min_dis:
                min_dis=tmp_min_dis
            min_dis_total+=tmp_min_dis
            for i in range(1,len(obs)):
                if obs[i][3]<0:
                    leagle=False
            if obs[0][2]>self.orca_policy.lane*self.orca_policy.lane_length+(-self.orca_policy.lane_length/2) and \
                obs[0][2]<self.orca_policy.lane*self.orca_policy.lane_length+(self.orca_policy.lane_length/2):
                in_target_lan_count+=1


            # print('action ',action, type(action))
            # print('obs ',obs)
            # print(obs[0][2],-self.orca_policy.lane_length/2,3*self.orca_policy.lane_length-self.orca_policy.lane_length/2)
            if obs[0][2]<-self.orca_policy.lane_length/2-1 or obs[0][2]>1+3*self.orca_policy.lane_length-self.orca_policy.lane_length/2:
                self.done=True
                crash=True

            if self.done:
                if info["crashed"] == True:
                    # print("crash")
                    crash=True
                else:
                    a=0
                    # print("done")
                break

            #get rl action in current env
            rl_action=self.get_rl_action(model,obs)
            orca_action,vel_speed=self.get_orca_action(obs)

            # print(count,' rl ',rl_action,' orca ',orca_action)
            # print('------predict time -------')
            #predict env at time=self.tau later
            predict_env_obs=self.env_predict(rl_action,obs,self.tau,count*self.dt)

            #get action in predict env and check, whether action is danger
            predict_orca_action,pre_vel_speed = self.get_orca_action(predict_env_obs)

            if self.danger_action(predict_env_obs,pre_vel_speed):
                # print('danger, choose ORCA action')
                action=orca_action

            else:
                rl_count+=1
                # print('save, choose RL action')
                action=rl_action
            # print('final action ',action)
            self.env.render()


        # print('----------------------------------------------------------')
        diff=int( self.predict_time/self.dt)
        # print(diff)
        for i in range(len(self.pose)):
            if i-diff<0:
                continue
            else:
                a=0
                # print(self.pose[i][1],' --- ',self.pred[i-diff][1])

        keep_in_target_lane_rate=in_target_lan_count/count
        avg_speed=speed_total/count
        #crash
        #min_dis
        avg_min_dis=min_dis_total/count
        return keep_in_target_lane_rate,avg_speed,crash,min_dis,avg_min_dis,count,leagle,rl_count/count


def main():

   new_highway_orca=SwitchLogic(18)
   new_highway_orca.run()

def anylize_test(dep=2.0):
    print('midu ',dep)
    total_keep_in_target_lane_rate=0
    total_avg_speed=0
    total_crash=0
    total_min_dis=0
    total_avg_min_dis=0
    total_count=0
    total_rl=0
    count=0
    for seed in range(1,51):
        new_highway_orca=SwitchLogic(seed)
        new_highway_orca.config['vehicles_density']=dep
        new_highway_orca.reinit()
        # print()
        tmp_keep_in_target_lane_rate, tmp_avg_speed, tmp_crash, tmp_min_dis, tmp_avg_min_dis,tmp_count,tmp_leagle,tmp_rl_rate=new_highway_orca.run()
        if tmp_leagle==False:
            print("illeagle     ===========          ")
            continue

        print('seed ',seed,' tar lane rate %.2f' % (tmp_keep_in_target_lane_rate),' avg speed %.2f' % (tmp_avg_speed),' crash ',tmp_crash
              ,' min_dis %.2f' % (tmp_min_dis),' avg min %.2f' % (tmp_avg_min_dis),' alive time %.2f' % (tmp_count),' rl rate %.2f' % (tmp_rl_rate))
        total_rl+=tmp_rl_rate
        if tmp_crash==True:
            total_crash+=1
        count+=1
        total_count+=tmp_count
        total_avg_min_dis+=tmp_avg_min_dis
        total_min_dis+=tmp_min_dis
        total_avg_speed+=tmp_avg_speed
        total_keep_in_target_lane_rate +=tmp_keep_in_target_lane_rate
    print('====================================================')

    print(' tar lane rate %.2f' % (total_keep_in_target_lane_rate/count),' avg speed %.2f' % (total_avg_speed/count)
          ,' crash %.2f' % (total_crash/count),total_crash,count,' min_dis %.2f' % (total_min_dis/count),
          ' avg min %.2f' % (total_avg_min_dis/count),' avg alive time %.2f' % (total_count/count),' avg rl time %.2f' % (total_rl/count))




if __name__ == '__main__':

    # main()
    for dep in [1,1.5,2]:
        print('========================================================================================')
        print(dep)
        anylize_test(dep)
