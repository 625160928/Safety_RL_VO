import math
import random

from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator

from npc_car.npc_car import NPCCar

#基本变道环境类
class ChangeLaneEnv():
    #model 为车辆模型，详情参考simulate car model
    #init lane 是车辆出生点车道
    #seed 随机数种子
    def __init__(self,model,init_lane=0,seed=0):
        #车道数量
        self._lane_number=3
        #单车道宽度
        self._lane_length=3
        #npc车辆列表，所有在环境里面正在迭代更新的车辆都会在这个数组里
        #npc列表里存的是 NPCCar 类
        self._npc_car_list=[]
        #观察范围
        self._obs_range=30
        #时间戳
        self._time=0
        #随机数种子，如果之后需要复现场景可以通过随机数种子重复试验
        self._random_seed=seed
        #时间步长
        self._step=0.1
        #npc计划列表
        #存的是 PreCreateCar 类
        self._create_plan=[]
        #仿真时间上限
        self._simulate_max_time=500
        #设置随机数种子
        random.seed(self._random_seed)
        #设置玩家车辆模型
        self._player_model=model
        #设置玩家车辆位置
        self._player_model.set_position(init_lane * self._lane_length + self._lane_length / 2, 0, math.pi/2)

    def draw_env(self,keep=False):
        plt.clf()
        # plt.figure(dpi=70, figsize=(self._lane_number*self._lane_length, self._obs_range*2))
        x_major_locator = MultipleLocator(1)
        y_major_locator = MultipleLocator(10)
        ax = plt.gca()
        # ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)
        #通过玩家位置设置绘图区域
        _,player_position_y,_,_,_=self._player_model.get_infomation()
        #设置显示范围
        plt.axis([0,self._lane_number*self._lane_length, player_position_y-self._obs_range, player_position_y+self._obs_range])
        #绘制玩家车辆
        self._player_model.draw_car(color='red')
        #绘制车道线
        for i in range(0,self._lane_number):
            plt.plot([i*self._lane_length,i*self._lane_length],[player_position_y-self._obs_range, player_position_y+self._obs_range],color='grey')
        #绘制npc车辆
        for car in self._npc_car_list:
            car.model.draw_car(color='black')
        #显示方法
        if keep==False:
            plt.pause(0.1)
        else:
            plt.show()

    #将计划里应该出现的车辆添加至npc车辆列表
    #添加计划里的车辆
    def _update_create_npc_car(self):
        for plan in self._create_plan:
            #只有达到预定时间的npc车辆才会被创建
            if self._time > plan.create_time:
                #设置npc策略
                strategy = plan.strategy
                # 设置npc仿真模型
                car_model = plan.car_model
                #通过当前玩家的位置设置npc的刷新位置
                _, player_position_y, _, _, _ = self._player_model.get_infomation()
                # 在仿真模型中设置npc车辆位置
                car_model.set_position(plan.lane * self._lane_length + self._lane_length / 2,
                                       player_position_y + self._obs_range + 10, math.pi / 2)
                # 设置npc车辆
                npc_car = NPCCar(car_model, strategy)
                self._npc_car_list.append(npc_car)
                self._create_plan.remove(plan)
                print("new npc ", plan.lane * self._lane_length + self._lane_length / 2,
                      player_position_y + self._obs_range + 10, math.pi / 2)

    # 通过npc的策略更新npc的位置
    def _update_npc_car(self):
        obs = self.obs()
        for npc_car in self._npc_car_list:
            v, w = npc_car.strategy.get_control(self._time, obs)
            npc_car.model.update(v, w)

    #迭代
    def update(self,v,w):
        #更新时间戳
        self._time+=self._step
        #根据时间戳创建计划车辆
        self._update_create_npc_car()
        #更新npc车辆的位置
        self._update_npc_car()
        #更新玩家的位置
        self._player_model.update(v, w)

    #观察周围环境
    #这部分需要根据实际情况进行更改，需要讨论
    def obs(self):
        player_position_x,player_position_y,player_position_theta,v,w=self._player_model.get_infomation()
        return self._time,player_position_x,player_position_y,player_position_theta,v,w





if __name__ == "__main__":
    env=ChangeLaneEnv()