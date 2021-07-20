import math


class StanleyControl():
    def __init__(self,L,dt,car_steer_limit):

        # stanley method参数
        self.__stanley_method_k = 0.5  # 增益参数 2
        self.__L = L  # 车辆轴距，单位：m
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


    def __stanley_control(self, x, y, theta, v, route_x, route_y, route_theta, pind):
        while theta < -math.pi:
            theta += 2 * math.pi
        while theta > math.pi:
            theta -= 2 * math.pi
        # print('--------')
        # print('in stanley_control ox,oy,theta = ', x, y, theta * 180 / math.pi, ' v = ', v, ' pind = ',
        #       pind)  # cx, cy, ch,

        ind = self.__calc_target_index(x, y, route_x, route_y)

        if pind >= ind:
            ind = pind

        if ind < len(route_x):
            tx = route_x[ind]
            ty = route_y[ind]
            th = route_theta[ind]
        else:
            tx = route_x[-1]
            ty = route_y[-1]
            th = route_theta[-1]
            ind = len(route_x) - 1

        # 计算横向误差
        if ((x - tx) * th - (y - ty)) > 0:
            error = abs(math.sqrt((x - tx) ** 2 + (y - ty) ** 2))
        else:
            error = -abs(math.sqrt((x - tx) ** 2 + (y - ty) ** 2))


        delta = route_theta[ind] - theta + math.atan2(self.__stanley_method_k * error, v)
        # print("error ",math.atan2(self.stanley_method_k * error, v)*180/math.pi,'--',self.stanley_method_k * error,v)
        # print('theta',(route_theta[ind] - theta )*180/math.pi,'--',route_theta[ind]*180/math.pi,  theta*180/math.pi )


        if delta > self.__car_steer_limit:
            delta = self.__car_steer_limit
        elif delta < - self.__car_steer_limit:
            delta = - self.__car_steer_limit

        return delta, ind

    def control(self, x, y, theta, v, route_x, route_y, route_theta,pind, ai):

        delta, ind =self.__stanley_control( x, y, theta, v, route_x, route_y, route_theta, pind)

        now_car_speed, w = self.__get_vw_from_ai_di(v, ai, delta)

        return now_car_speed, w,ind


