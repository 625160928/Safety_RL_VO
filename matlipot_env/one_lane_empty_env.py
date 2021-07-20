import math

import matplotlib.pyplot as plt

from base_lane_env import ChangeLaneEnv
import random
from npc_car.npc_car_strategy.strategy_speed_down_and_stay import StrategySpeedDownAndStay
from npc_car.pre_create_car import  PreCreateCar
from npc_car.car_model import simulate_car_model
from npc_car.npc_car import NPCCar
"""
OneLaneEmptyEnv(ChangeLaneEnv)
该类为除了empty lane 没有车辆，其他车道都在某个时间点附近刷新一部车辆
刷新的npc车辆会在玩家车辆的前方
车辆运行策略为先低速运行一段时间保证拉近与玩家车辆的距离，随后恢复高速匀速运动

"""
class OneLaneEmptyEnv(ChangeLaneEnv):
    # model 为车辆模型，详情参考simulate car model
    # init lane 是车辆出生点车道
    #empty lane 是空车道的位置
    # seed 随机数种子
    def __init__(self,model,init_lane=0,empty_lane=0,seed=0):
        ChangeLaneEnv.__init__(self,model,init_lane,seed)
        #设置npc车辆的出现时间
        self._npc_appear_time=random.randint(0,10)
        self._npc_appear_time=2

        #车辆低速运行的时间长度
        self.npc_speed_down_time=2
        #往计划列表里添加npc车辆计划
        for i in range(0,self._lane_number):
            #除了指定的车道不刷新车辆，其他的都要添加npc车辆
            if i!= empty_lane:
                #随机设置车辆的出现时间
                npc_appear_time=self._npc_appear_time+random.randint(0,10)/5

                # 设置仿真模型
                car_model= simulate_car_model.create_car_example()

                # 设置npc车辆的目标速度
                npc_tar_speed = 50

                # 设置恢复恒速的时间
                npc_speed_down_time = npc_appear_time + self.npc_speed_down_time

                #车辆存活时间
                # alive_time=self._simulate_max_time
                alive_time=npc_speed_down_time +20

                # 设置车辆移动策略
                strategy = StrategySpeedDownAndStay(npc_tar_speed, npc_speed_down_time, alive_time)

                #添加计划
                self._create_plan.append(PreCreateCar(self._npc_appear_time,alive_time , i,
                                                      strategy, car_model))










if __name__ == "__main__":
    player_model= simulate_car_model.create_car_example()
    env=OneLaneEmptyEnv(model=player_model, init_lane=0, empty_lane=1, seed=10)

    for i in range(1000):
        env.update(50,0)

        print(env.obs())

        env.draw_env(keep=False)