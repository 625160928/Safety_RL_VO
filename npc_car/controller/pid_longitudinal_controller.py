import numpy as np
from collections import deque
import math
from geometry_msgs.msg import Point  # pylint: disable=import-error

class PIDLongitudinalController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

    #这个原先只能做正向跟踪，只控制油门
    #将控制下限调整为-1后经过处理转换为刹车变为全向控制
    def run_step(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        previous_error = self.error
        self.error = target_speed - current_speed
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -40.0, 40.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        return np.clip(output, -1.0, 1.0)

    def combine_step(self, target_speed, current_speed):

        if target_speed>0:
            throttle=self.run_step(target_speed, current_speed)
            reverse=False
        else:
            throttle=self.run_step(-target_speed, -current_speed)
            reverse=True


        if throttle>0:
            return throttle,0,reverse
        else:
            return 0,-throttle,reverse


