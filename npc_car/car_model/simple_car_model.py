import math
import numpy as np
from matplotlib import pyplot as plt


class SimpleCarModel():
    def __init__(self, x, y, theta, v=0, w=0, cl=None, cw=None, min_r=None, wl=None,
                 hx=None, step=None):
        self._car_l = cl
        self.__car_w = cw
        self.__min_turning_radiu = min_r
        self.__wheel_dis = wl
        self._hou_xuan = hx
        self._step = step

        self._position_x = x
        self._position_y = y
        self._position_theta = theta
        self.mode = ""
        self._v = v
        self._w = w

    def set_infomation(self, cl, cw, min_r, wl, hx, step):
        self._car_l = cl
        self.__car_w = cw
        self.__min_turning_radiu = min_r
        self.__wheel_dis = wl
        self._hou_xuan = hx
        self._step = step

    def get_infomation(self):
        return self._position_x, self._position_y, self._position_theta, self._v, self._w

    def set_position(self, x, y, theta):
        self._position_x = x
        self._position_y = y
        self._position_theta = theta
        return self._position_x, self._position_y, self._position_theta

    def draw_car(self, color='black'):

        whx1 = self._position_x - math.sin(self._position_theta) * self.__car_w / 2
        whx2 = self._position_x + math.sin(self._position_theta) * self.__car_w / 2
        why1 = self._position_y + math.cos(self._position_theta) * self.__car_w / 2
        why2 = self._position_y - math.cos(self._position_theta) * self.__car_w / 2

        # 车后轮的轴线
        x1 = [whx1, whx2]
        y1 = [why1, why2]
        plt.plot(x1, y1, color)
        # 车边框
        # 车左边框
        x1 = [whx1 - (self._hou_xuan) * math.cos(self._position_theta),
              whx1 + (self._car_l - self._hou_xuan) * math.cos(self._position_theta)]
        y1 = [why1 - (self._hou_xuan) * math.sin(self._position_theta),
              why1 + (self._car_l - self._hou_xuan) * math.sin(self._position_theta)]
        plt.plot(x1, y1, color)
        # 车右边框
        x1 = [whx2 - (self._hou_xuan) * math.cos(self._position_theta),
              whx2 + (self._car_l - self._hou_xuan) * math.cos(self._position_theta)]
        y1 = [why2 - (self._hou_xuan) * math.sin(self._position_theta),
              why2 + (self._car_l - self._hou_xuan) * math.sin(self._position_theta)]
        plt.plot(x1, y1, color)
        # 车尾边框
        x1 = [whx1 - (self._hou_xuan) * math.cos(self._position_theta),
              whx2 - (self._hou_xuan) * math.cos(self._position_theta)]
        y1 = [why1 - (self._hou_xuan) * math.sin(self._position_theta),
              why2 - (self._hou_xuan) * math.sin(self._position_theta)]
        plt.plot(x1, y1, color)
        # 车头边框
        x1 = [whx1 + (self._car_l - self._hou_xuan) * math.cos(self._position_theta),
              whx2 + (self._car_l - self._hou_xuan) * math.cos(self._position_theta)]
        y1 = [why1 + (self._car_l - self._hou_xuan) * math.sin(self._position_theta),
              why2 + (self._car_l - self._hou_xuan) * math.sin(self._position_theta)]
        plt.plot(x1, y1, color)


    def get_corners(self):
        corners=[]
        whx1 = self._position_x - math.sin(self._position_theta) * self.__car_w / 2
        whx2 = self._position_x + math.sin(self._position_theta) * self.__car_w / 2
        why1 = self._position_y + math.cos(self._position_theta) * self.__car_w / 2
        why2 = self._position_y - math.cos(self._position_theta) * self.__car_w / 2

        # 车边框
        # 车左边框
        x1 = [whx1 - (self._hou_xuan) * math.cos(self._position_theta),
              whx1 + (self._car_l - self._hou_xuan) * math.cos(self._position_theta)]
        y1 = [why1 - (self._hou_xuan) * math.sin(self._position_theta),
              why1 + (self._car_l - self._hou_xuan) * math.sin(self._position_theta)]

        corners.append([x1[0],y1[0]])
        corners.append([x1[1],y1[1]])
        # 车右边框
        x1 = [whx2 - (self._hou_xuan) * math.cos(self._position_theta),
              whx2 + (self._car_l - self._hou_xuan) * math.cos(self._position_theta)]
        y1 = [why2 - (self._hou_xuan) * math.sin(self._position_theta),
              why2 + (self._car_l - self._hou_xuan) * math.sin(self._position_theta)]

        corners.append([x1[0],y1[0]])
        corners.append([x1[1],y1[1]])
        return corners







