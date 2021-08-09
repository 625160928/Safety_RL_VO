import math
import numpy as np
from matplotlib import pyplot as plt
from npc_car.car_model.simple_car_model import SimpleCarModel

class CarModel(SimpleCarModel):
    def __init__(self,x,y,theta,v=0,w=0, cl=None, cw=None, min_r=None, wl=None,
                 hx=None, step=None):
        SimpleCarModel.__init__(self, x, y, theta, v, w, cl, cw, min_r, wl, hx, step)

        self.mode="simulate"


    def update(self,v,w):
        self._position_x = self._position_x + v * np.cos(self._position_theta) * self._step
        self._position_y = self._position_y + v * np.sin(self._position_theta) * self._step
        self._position_theta = self._position_theta + w * self._step
        self._v=v
        self._w=w


    def stop(self):
        self._v=0
        self._w=0
        return

    def reset_car(self):
        self._position_x = self.init_x
        self._position_y = self.init_y
        self._position_theta = self.init_theta
        self._v=self.init_v
        self._w=self.init_w

def create_car_example():
    parking_l = 5.3
    road_w = 2.86 * 1.6  # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4.7
    car_w = 1.7
    min_turning_radiu = 5.3
    wheel_dis = 2.6
    road_l = 3 * parking_l
    step = 0.1
    hou_xuan = (car_l - wheel_dis) / 2
    car0=CarModel(0, 0, 0)
    #cl, cw, min_r, wl, hx, step
    car0.set_infomation(car_l,car_w,min_turning_radiu,wheel_dis,hou_xuan,step)
    return car0


if __name__ == "__main__":
    parking_l = 5.3
    road_w = 2.86 * 1.6  # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4.7
    car_w = 1.7
    min_turning_radiu = 5.3
    wheel_dis = 2.6
    road_l = 3 * parking_l
    step = 0.1
    hou_xuan = (car_l - wheel_dis) / 2
    car0=CarModel(0, 0, 0)
    #cl, cw, min_r, wl, hx, step
    car0.set_infomation(car_l,car_w,min_turning_radiu,wheel_dis,hou_xuan,step)

    import random
    random.seed(0)
    for i in range(100):
        v=random.randint(0,10)/10
        w=random.randint(-3,3)/10
        plt.xlim(-10,10)
        plt.ylim(-10,10)
        car0.update(v,w)
        car0.draw_car()
        plt.pause(step)
        plt.clf()
        print("v w ",v,w)
        print("position", car0.get_infomation())




