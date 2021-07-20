import math
import random

from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator

from npc_car.npc_car import NPCCar


class ChangeLaneEnv():
    def __init__(self,model,init_lane=0,seed=0):
        self._lane_number=3
        self._lane_length=3
        self._npc_car_list=[]
        self._obs_range=10
        self._time=0
        self._random_seed=seed
        self._step=0.1
        self._create_plan=[]
        self._simulate_max_time=500
        random.seed=self._random_seed
        self._player_model=model
        self._player_model.set_position(init_lane * self._lane_length + self._lane_length / 2, 0, math.pi/2)

    def draw_env(self,keep=False):
        plt.clf()
        # plt.figure(dpi=70, figsize=(self._lane_number*self._lane_length, self._obs_range*2))
        x_major_locator = MultipleLocator(1)
        y_major_locator = MultipleLocator(1)
        ax = plt.gca()
        # ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)
        #通过玩家位置设置绘图区域
        _,player_position_y,_,_,_=self._player_model.get_infomation()
        plt.axis([0,self._lane_number*self._lane_length, player_position_y-self._obs_range, player_position_y+self._obs_range])
        self._player_model.draw_car(color='red')
        for i in range(0,self._lane_number):
            plt.plot([i*self._lane_length,i*self._lane_length],[player_position_y-self._obs_range, player_position_y+self._obs_range],color='grey')
        for car in self._npc_car_list:
            car.model.draw_car()
        if keep==False:
            plt.pause(0.1)
        else:
            plt.show()

    #将计划中的
    def _update_create_npc_car(self):
        for plan in self._create_plan:
            if self._time > plan.create_time:
                strategy = plan.strategy
                # 设置仿真模型
                car_model = plan.car_model
                _, player_position_y, _, _, _ = self._player_model.get_infomation()
                # 在仿真模型中设置车辆位置
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

    def obs(self):
        player_position_x,player_position_y,player_position_theta,v,w=self._player_model.get_infomation()
        return self._time,player_position_x,player_position_y,player_position_theta,v,w





if __name__ == "__main__":
    env=ChangeLaneEnv()