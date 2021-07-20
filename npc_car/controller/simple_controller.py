import math

import numpy as np


class SimpleThrolleController():  # pylint: disable=too-few-public-methods
    def __init__(self):
        self.force=1.7

    def combine_step(self, target_v, current_speed):

        if target_v==0:
            return 0,1,True

        force=1.7

        if (target_v*current_speed<0):
            if target_v>0:
                car_reverse = False
                if target_v>current_speed+1:
                    car_brake=1
                    car_throttle=0
                else:
                    car_brake=0
                    car_throttle =(target_v-current_speed)/force
            else:
                car_reverse = True

                if target_v<current_speed-1:
                    car_brake=1
                    car_throttle=0
                else:
                    car_brake=0
                    car_throttle =(current_speed-target_v)/force
        else:
            if target_v>0:
                car_reverse=False
            else:
                car_reverse=True

            abs_current_speed=abs(current_speed)
            abs_target_v=abs(target_v)
            if abs_target_v>abs_current_speed:
                car_brake=0
                car_throttle=(abs_target_v-abs_current_speed)/force
            else:
                car_brake=(abs_current_speed-abs_target_v)/force
                car_throttle=0


        if car_throttle<0:
            car_throttle=0
        if car_throttle>1:
            car_throttle=1

        return car_throttle,car_brake,car_reverse

class SimpleSteerController():
    def __init__(self):
        #  限制车轮转角
        self.__car_steer_limit = math.pi / 6


    def run_step(self,target_w,current_speed):
        send_w = target_w / self.__car_steer_limit
        if current_speed > 0:
            send_w = -send_w

        send_w = np.clip(send_w, -1.0, 1.0)
        return send_w
