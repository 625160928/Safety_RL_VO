import math


class PurePursuitControl():
    def __init__(self,L,dt,car_steer_limit):
        self.__pure_pursuit_long_see = 100
        self.__pure_pursuit_dis = 0.3
        self.__L = L  # 车辆轴距，单位：m
        self.__pure_pursuit_k_f = 0.2  # 前视距离系数
        self.__pure_pursuit_Lfc = 2  # 前视距离
        self.__pure_pursuit_min_r = 0.25
        self.__dt=dt

        #  限制车轮转角
        self.__car_steer_limit = car_steer_limit


    #将加速度与横向偏移变成车辆速度与角速度
    def __get_vw_from_ai_di(self, car_speed, acc, delta):
        v = car_speed + acc * self.__dt
        w= car_speed / self.__L * math.tan(delta)
        return v,w

    # 搜索最临近的路点
    def __calc_target_index(self, x, y, route_x, route_y):
        dx = [x - icx for icx in route_x]
        dy = [y - icy for icy in route_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        return ind

    def __pure_pursuit_calc_target_index(self, x, y, v, route_x, route_y):
        # pure_pursuit搜索最临近的路点
        ind = self.__calc_target_index(x, y, route_x, route_y)
        L_ = 0.0

        Lf = self.__pure_pursuit_k_f * v + self.__pure_pursuit_Lfc
        max_count = len(route_x) / 20
        while Lf > L_ and (ind + 1) < len(route_x):
            dx = route_x[ind + 1] - route_x[ind]
            dy = route_x[ind + 1] - route_x[ind]
            L_ += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
            max_count -= 1
            if max_count <= 0:
                break

        return ind


    def __pure_pursuit_control(self, x, y, theta, v, route_x, route_y, pind):

        ind = self.__pure_pursuit_calc_target_index(x, y, v, route_x, route_y)

        if pind >= ind:
            ind = pind

        if ind < len(route_x):
            tx = route_x[ind]
            ty = route_y[ind]
        else:
            tx = route_x[-1]
            ty = route_y[-1]
            ind = len(route_x) - 1

        alpha = math.atan2(ty - y, tx - x) - theta

        if v < 0:  # back
            alpha = math.pi - alpha

        Lf = self.__pure_pursuit_k_f * v + self.__pure_pursuit_Lfc

        delta = math.atan2(2.0 * self.__L * math.sin(alpha) / Lf, self.__pure_pursuit_min_r)

        if delta > self.__car_steer_limit:
            delta = self.__car_steer_limit
        elif delta < - self.__car_steer_limit:
            delta = - self.__car_steer_limit

        return delta, ind


    def control(self, x, y, theta, v, route_x, route_y, route_theta,pind, ai):
        delta, ind =self.__pure_pursuit_control( x, y, theta, v, route_x, route_y,pind)

        now_car_speed, w = self.__get_vw_from_ai_di(v, ai, delta)

        return now_car_speed, w,ind
