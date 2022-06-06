import math

import numpy
import yaml
import matplotlib.pyplot as plt


from orca_src.road_orca import RoadOrca
from env.highway_sim_env import HighwaySimulation
from env.grid_map import GridMap
from orca_src.tubianxing import Is_rec_collide
from orca_src import draw_picture
"""
切换逻辑主程序部分
"""

class SwitchLogic():
    def __init__(self,env=None):
        config=env.parm_config

        #车辆模型参数
        self.car_steer_limit = config['car_steer_limit']

        self.switch_danger_theta = config['switch_danger_theta']
        self.switch_danger_dis = config['switch_danger_dis']
        self.default_method=config['default_method']
        self.env=env

        self.orca_policy=RoadOrca(config=config, method=self.default_method)
        self.rl_policy=RoadOrca(config=config, method='avo')

    #et orca_src action from orca_src
    def get_orca_action(self,obs,method=None,map=None):
        return self.orca_policy.get_action(obs, method,map)

    def get_rl_action(self,model,obs,road_map,method):
        orca_action, vel_speed = self.orca_policy.get_action(obs, method, road_map)
        return orca_action

    def danger_action(self, pre_env_obs, orca_speed):

        now_speed_x=pre_env_obs[0][3]
        now_speed_y=pre_env_obs[0][4]
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
        # print('predict now speed ',now_speed_x,now_speed_y,' orca_src ',orca_speed,' derta_speed ',derta_speed_x,derta_speed_y)
        # print('predict position theta ',math.atan2(position_sin_theta,position_cos_theta),' orca_src v theta ',math.atan2(orca_speed[1],orca_speed[0]))

        # print(' danger angle ',abs(math.acos(derta_cos_theta))*180/math.pi,' limit  ',self.switch_danger_theta*180/math.pi)
        # if abs(math.acos(derta_cos_theta))>self.switch_danger_theta:
        #     return True
        # print('danger dis ',math.hypot(derta_speed_x,derta_speed_y),self.switch_danger_dis)
        if math.hypot(derta_speed_x,derta_speed_y)>self.switch_danger_dis:
            return True

        # return True
        return False

    def orac_t_danger_action(self, obs):

        my_agent_area=self.orca_policy.get_position_avaliable_set(obs[0], t=1)
        others_agent_area=[]
        for obj in obs[1:]:
            ag=self.orca_policy.get_position_avaliable_set(obj, t=1)
            others_agent_area.append(ag)

        plt.clf()
        draw_picture.draw_tubianxing(my_agent_area,color='red')
        for obj in others_agent_area:
            draw_picture.draw_tubianxing(obj)
        plt.pause(0.1)

        for area in others_agent_area:
            if Is_rec_collide(area, my_agent_area)==True:
                # print('type - 1 ',area, my_agent_area)
                return True
        my_agent_area2 = self.orca_policy.get_position_avaliable_set(obs[0], t=2)
        others_agent_area2 = []
        for obj in obs[1:]:
            ag = self.orca_policy.get_position_avaliable_set(obj, t=2)
            others_agent_area2.append(ag)

        for area in others_agent_area2:
            if Is_rec_collide(area, my_agent_area2)==True:
                # print('type - 2 ',area, my_agent_area2)
                return True
        return False

    def run(self,switch_count=9999,switch_method='orca'):

        action = (0, 0)
        model=None

        # ========================下面是反馈参数记录
        crash=False
        leagle=True
        in_target_lan_count=0
        speed_total=0
        min_dis_total=0
        min_coll_dis=99999999999999
        switch_times=0
        count = 0
        rl_count=0
        old='rl'
        # ========================上面是反馈参数记录


        # ========================下面是map

        road_map = GridMap()
        road_map.x_length = 200
        road_map.y_length = 30
        road_map.xy_resolusion = 1
        road_map.map = road_map.gengerate_map()
        road_map.zero_y = -5
        road_map.zero_x = 0
        road_map.zero_theta = 0
        road_map.reset_space_change()

        for i in range(road_map.map.shape[0]):
            for j in range(road_map.map.shape[1]):
                if j*road_map.xy_resolusion+road_map.zero_y <-self.env.parm_config['lane_length']/2\
                        or j*road_map.xy_resolusion+road_map.zero_y >self.env.parm_config['lane_length']*self.env.parm_config['lanes_count']-self.env.parm_config['lane_length']/2:

                    road_map.map[i][j]=1
        # ========================上面是map

        while not self.env.done:
            count += 1

            obs, reward, self.done, info = self.env.step(action)

            # ========================下面是反馈参数记录
            #collect speed info
            speed_total+=obs[0][3]

            tmp_min_dis=self.env.get_cloest_distance(obs)
            if tmp_min_dis<min_coll_dis:
                min_coll_dis=tmp_min_dis
            min_dis_total+=tmp_min_dis
            for i in range(1,len(obs)):
                if obs[i][3]<0:
                    leagle=False

            if obs[0][2]>self.orca_policy.lane*self.orca_policy.lane_length+(-self.orca_policy.lane_length/2) and \
                obs[0][2]<self.orca_policy.lane*self.orca_policy.lane_length+(self.orca_policy.lane_length/2):
                in_target_lan_count+=1
            # =================反馈参数记录

            #添加公路边界的碰撞
            if obs[0][2]<-self.orca_policy.lane_length/2-1 or obs[0][2]>1+3*self.orca_policy.lane_length-self.orca_policy.lane_length/2:
                info["crashed"] = True

            if self.done:
                if info["crashed"] == True:
                    crash=True
                break

            #update map
            road_map.zero_x=obs[0][1]-100
            road_map.reset_space_change()


            #获取当前时刻下的orca操作与rl操作
            rl_action=self.get_rl_action(model,obs,road_map,self.default_method)

            if count<switch_count:
                orca_action,vel_speed=self.get_orca_action(obs,self.default_method,road_map)
            else:
                orca_action,vel_speed=self.get_orca_action(obs,switch_method,road_map)

            # #预测一段时间后的状态
            # predict_env_obs=self.env.env_predict(rl_action,obs)
            #
            # #获取一段时间后的orca操作，用来作为危险判断参考
            # if count<switch_count:
            #     predict_orca_action,pre_vel_speed = self.get_orca_action(predict_env_obs,map=road_map)
            # else:
            #     predict_orca_action,pre_vel_speed = self.get_orca_action(predict_env_obs,method=switch_method,map=road_map)

            #危险判断，判断一段时间后的状态是否安全
            if self.orac_t_danger_action(obs=obs):
                print('danger, choose ORCA action')
                action=orca_action
                if old=='rl':
                    switch_times+=1
                    old="orca_src"

            else:
                print('save, choose RL action')
                rl_count+=1
                if old=='orca_src':
                    switch_times+=1
                    old="rl"
                action=rl_action

            #环境迭代
            self.env.render()

        #=================反馈参数记录
        keep_in_target_lane_rate=in_target_lan_count/count
        avg_speed=speed_total/count
        avg_min_dis=min_dis_total/count
        #=================反馈参数记录

        return keep_in_target_lane_rate,avg_speed,crash,min_coll_dis,avg_min_dis,count,leagle,rl_count/count,switch_times


def main():
    f = open(r'highway_config.yaml', 'r', encoding='utf-8')
    result = f.read()
    config = yaml.load(result,Loader=yaml.CLoader)





    sim_env=HighwaySimulation(env_name="merge-v0",config=config)
    # sim_env=HighwaySimulation(env_name="highway-v0",config=config)
    new_highway_orca=SwitchLogic(sim_env)
    new_highway_orca.run()



if __name__ == '__main__':

    main()