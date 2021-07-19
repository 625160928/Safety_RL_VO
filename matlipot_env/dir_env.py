import math

import matplotlib.pyplot as plt

from base_lane_env import ChangeLaneEnv
import random
from npc_car.npc_car_strategy.strategy_speed_down_and_stay import StrategySpeedDownAndStay
from npc_car.pre_create_car import  PreCreateCar
from npc_car.car_model import simulate_car_model
from npc_car.npc_car import NPCCar

class DirEnv(ChangeLaneEnv):
    def __init__(self,model,init_lane=0,seed=0):
        ChangeLaneEnv.__init__(self,model,init_lane,seed)

        self._npc_appear_time=random.randint(0,10)
        self._npc_appear_time=1

        for i in range(0,self._lane_number):
            if i!= init_lane:
                car_model= simulate_car_model.create_car_example()

                self._create_plan.append(PreCreateCar(self._npc_appear_time, self._simulate_max_time - self._npc_appear_time, i,
                                                      StrategySpeedDownAndStay, car_model))


    def __update_create_npc_car(self):
        for plan in self._create_plan:
            if self._time>plan.create_time:
                #设置npc车辆的目标速度
                npc_tar_speed=50
                #设置恢复恒速的时间
                npc_speed_down_time=2
                #设置车辆移动策略
                strategy=plan.strategy(npc_tar_speed,npc_speed_down_time,plan.alive_time)
                #设置仿真模型
                car_model=plan.car_model
                _,player_position_y,_,_,_=self._player_model.get_infomation()
                #在仿真模型中设置车辆位置
                car_model.set_position(plan.lane * self._lane_length + self._lane_length / 2 , player_position_y+self._obs_range+10,math.pi/2)
                #设置npc车辆
                npc_car=NPCCar(car_model,strategy)
                self._npc_car_list.append(npc_car)
                print("new npc ",plan.lane * self._lane_length + self._lane_length / 2 , player_position_y+self._obs_range+10,math.pi/2)
                self._create_plan.remove(plan)



    #通过npc的策略更新npc的位置
    def __update_npc_car(self):
        obs=self.obs()
        for npc_car in self._npc_car_list:
            v,w=npc_car.strategy.get_control(self._time,obs)
            npc_car.model.update(v,w)




    def update(self,v,w):
        self._time+=self._step
        self.__update_create_npc_car()
        self.__update_npc_car()
        self._player_model.update(v, w)



if __name__ == "__main__":
    player_model= simulate_car_model.create_car_example()
    env=DirEnv(model=player_model,init_lane=1,seed=10)

    for i in range(1000):
        env.update(50,0)

        print(env.obs())

        env.draw_env(keep=False)