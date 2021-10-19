
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
from highway_sim_env import HighwaySimulation
# import torch as th
# from stable_baselines3 import PPO
# from torch.distributions import Categorical
# import torch
# import torch.nn as nn
# from torch.nn import functional as F

"""
切换逻辑主程序部分
"""

class SwitchLogic():
    def __init__(self,seed=0):
        #车辆模型参数
        self.car_steer_limit = math.pi / 3

        self.switch_danger_theta = math.pi / 6
        self.switch_danger_dis = 3.5

        self.orca_policy=HighWayOrca(seed,'avo')
        self.env=HighwaySimulation(10)

    #et car_orca action from car_orca
    def get_orca_action(self,obs,method=None):
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
        if method==None:
            new_vels, all_line = orca(agents[0], agents[1:], self.env.tau, self.env.dt,
                                      limit=[-2 + agents[0].radius / 2 + self.orca_policy.edge_remain,
                                             14 - agents[0].radius / 2 - self.orca_policy.edge_remain],method=self.orca_policy.method)
        else:
            new_vels, all_line = orca(agents[0], agents[1:], self.env.tau, self.env.dt,
                                      limit=[-2 + agents[0].radius / 2 + self.orca_policy.edge_remain,
                                             14 - agents[0].radius / 2 - self.orca_policy.edge_remain],method=method)


        # 将速度转换为动作指令
        action = self.orca_policy.change_vxvy_to_action(agents[0], new_vels)
        
        return action,new_vels

    def get_rl_action(self,model,obs):
        # action, _ = model.predict(obs)
        # print(action)
        return (0.5,-0.5)
        # return action

    def danger_action(self,env_obs,orca_speed):

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

    def run(self,switch_count=9999,switch_method='orca'):
        in_target_lan_count=0
        crash=False
        speed_total=0
        min_dis_total=0
        min_dis=99999999999999
        leagle=True

        model=0
        # model = PPO.load("./ppo-highway8")
        switch_times=0
        count = 0
        action = (0, 0)
        rl_count=0
        old='rl'

        # for-path
        carlist = []
        allcar = []
        # veh = self.env.road.vehicles
        # if veh:
        #     for v in veh:
        #         Ind = 0
        #         carlist.append([Ind, v.position[0], v.position[1], v.heading])

        # car_data = np.array([carlist])
        allcar.append(carlist)

        # end for-path

        while not self.env.done:
            # print('----------------------------------------------------------')
            count += 1
            # print('---------------------------------------------')
            # print('run count ', count,"  time = ",count*self.dt)
            # print('final action is ',action)

            obs, reward, self.done, info = self.env.step(action)
            # self.pose.append([count*self.dt,[obs[0][1],obs[0][2]]])
            # print("now pose is ",obs[0][1],obs[0][2],' speed is ',[obs[0][3],obs[0][4]])

            #collect speed info
            speed_total+=obs[0][3]

            tmp_min_dis=self.env.get_cloest_distance(obs)
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
            if count<switch_count:
                orca_action,vel_speed=self.get_orca_action(obs)
            else:
                orca_action,vel_speed=self.get_orca_action(obs,switch_method)

            # print(count,' rl ',rl_action,' orca ',orca_action)
            # print('------predict time -------')
            #predict env at time=self.tau later
            predict_env_obs=self.env.env_predict(rl_action,obs)

            #get action in predict env and check, whether action is danger

            if count<switch_count:
                predict_orca_action,pre_vel_speed = self.get_orca_action(predict_env_obs)
            else:
                predict_orca_action,pre_vel_speed = self.get_orca_action(predict_env_obs,switch_method)


            if self.danger_action(predict_env_obs,pre_vel_speed):
                # print('danger, choose ORCA action')
                action=orca_action
                if old=='rl':
                    switch_times+=1
                    old="orca"

            else:
                rl_count+=1
                if old=='orca':
                    switch_times+=1
                    old="rl"
                # print('save, choose RL action')
                action=rl_action
            # print('final action ',action)
            # action = orca_action

            # #  for-path
            # carlist = []
            # veh = self.env.road.vehicles
            # if veh:
            #     for v in veh:
            #         Ind = count
            #         carlist.append([Ind, v.position[0], v.position[1], v.heading])
            #
            # # car_data=hstack((car_data,[np.array(carlist)]))
            # allcar.append(carlist)
            # #  end for-path

            self.env.render()


        # # print('----------------------------------------------------------')
        # diff=int( self.predict_time/self.dt)
        # # print(diff)
        # for i in range(len(self.pose)):
        #     if i-diff<0:
        #         continue
        #     else:
        #         a=0
        #         # print(self.pose[i][1],' --- ',self.pred[i-diff][1])

        keep_in_target_lane_rate=in_target_lan_count/count
        avg_speed=speed_total/count
        #crash
        #min_dis
        avg_min_dis=min_dis_total/count

        car_data = np.array(allcar)
        np.save('car_data_mix.npy', car_data)

        return keep_in_target_lane_rate,avg_speed,crash,min_dis,avg_min_dis,count,leagle,rl_count/count,switch_times


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
    total_switch_times=0
    count=0


    #初始化使用的orca方法
    init_method='avo'

    # 在第switch_to_orca_count的时候强制切换成switch_methods方法
    switch_to_orca_count=999
    switch_methods='orca'

    for seed in range(1,51):
        new_highway_orca=SwitchLogic(seed)



        new_highway_orca.orca_policy.method=init_method   # chu shi xuan ze avo



        new_highway_orca.config['vehicles_density']=dep
        new_highway_orca.reinit()
        # print()
        tmp_keep_in_target_lane_rate, tmp_avg_speed, tmp_crash, tmp_min_dis, tmp_avg_min_dis,tmp_count,\
        tmp_leagle,tmp_rl_rate,tmp_switch_times=new_highway_orca.run(switch_count=switch_to_orca_count,switch_method=switch_methods)
        if tmp_leagle==False:
            print("illeagle     ===========          ")
            continue

        print('seed ',seed,' tar lane rate %.2f' % (tmp_keep_in_target_lane_rate),' avg speed %.2f' % (tmp_avg_speed),' crash ',tmp_crash
              ,' min_dis %.2f' % (tmp_min_dis),' avg min %.2f' % (tmp_avg_min_dis),' alive time %.2f' % (tmp_count),
              ' rl rate %.2f' % (tmp_rl_rate),' switch times %.2f' % (tmp_switch_times))
        total_rl+=tmp_rl_rate*tmp_count
        if tmp_crash==True:
            total_crash+=1
        count+=1
        total_count+=tmp_count
        total_avg_min_dis+=tmp_avg_min_dis
        total_switch_times+=tmp_switch_times
        total_min_dis+=tmp_min_dis
        total_avg_speed+=tmp_avg_speed
        total_keep_in_target_lane_rate +=tmp_keep_in_target_lane_rate
    print('====================================================')

    print(' tar lane rate %.2f' % (total_keep_in_target_lane_rate/count),' avg speed %.2f' % (total_avg_speed/count)
          ,' crash %.2f' % (total_crash/count),total_crash,count,' min_dis %.2f' % (total_min_dis/count),
          ' avg min %.2f' % (total_avg_min_dis/count),' avg alive time %.2f' % (total_count/count)
          ,' avg rl time %.2f' % (total_rl/total_count),' avg switch time %.2f' % (total_switch_times/count))




if __name__ == '__main__':

    main()
    # for dep in [1,1.5,2]:
    #     print('========================================================================================')
    #     print(dep)
    #     anylize_test(dep)
