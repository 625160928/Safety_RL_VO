import math

import matplotlib.pyplot as plt

from base_lane_env import ChangeLaneEnv
import random
from npc_car.npc_car_strategy.strategy_speed_down_and_stay import StrategySpeedDownAndStay
from npc_car.pre_create_car import  PreCreateCar
from npc_car.car_model import simulate_car_model
from npc_car.npc_car import NPCCar

class DirEnv(ChangeLaneEnv):
    def __init__(self,model,init_lane=0,empty_lane=0,seed=0):
        ChangeLaneEnv.__init__(self,model,init_lane,seed)

        self._npc_appear_time=random.randint(0,10)
        self._npc_appear_time=2
        self.npc_speed_down_time=2
        for i in range(0,self._lane_number):
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
                alive_time=self._simulate_max_time

                # 设置车辆移动策略
                strategy = StrategySpeedDownAndStay(npc_tar_speed, npc_speed_down_time, alive_time)

                self._create_plan.append(PreCreateCar(self._npc_appear_time,alive_time , i,
                                                      strategy, car_model))







    def update(self,v,w):
        self._time+=self._step
        self._update_create_npc_car()
        self._update_npc_car()
        self._player_model.update(v, w)



if __name__ == "__main__":
    player_model= simulate_car_model.create_car_example()
    env=DirEnv(model=player_model,init_lane=0,empty_lane=1,seed=10)

    for i in range(1000):
        env.update(50,0)

        print(env.obs())

        env.draw_env(keep=False)