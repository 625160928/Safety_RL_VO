import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from numpy import array, sqrt, copysign, dot
from numpy.linalg import det


from controller import pid_lateral_controller_angle
from orca_src.class_line import Line
from orca_src.Minkowski import Minkowski_sum
from orca_src.class_agent import Agent
from orca_src import Minkowski, tubianxing


class Orca():
    def __init__(self,method='orca'):
        self.car_steer_limit=math.pi/3
        self.acc = 0.5
        self.tau = 2
        self.dt=0.05
        self.prev = 20
        self.lane=1
        self.lane_length=4
        self.limit_range=[-20,34]

        self.save_dis = 1.5
        self.save_reward = 0.5

        # 速度P控制器系数 1
        self.Speed_Kp = 0.6
        self.car_radiu=3.2
        self.method=method
        self.edge_remain=0.3

        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)
        self.later_pid=pid_lateral_controller_angle.PIDLateralController(L=2.5, dt=self.dt, car_steer_limit=self.car_steer_limit, K_P=4, K_D=0.2, K_I=0)

    def _perp(self,a):
        return array((a[1], -a[0]))

    def _norm_sq(self,x):
        return dot(x, x)

    def _normalized(self,x):
        l = self._norm_sq(x)
        assert l > 0, (x, l)
        return x / sqrt(l)

    def _dist_sq(self,a, b):
        return self._norm_sq(b - a)

    def set_pref_v(self,current_x,current_y,goal_x,goal_y,v):
        theta = math.atan2(goal_y - current_y, goal_x - current_x)
        return  [math.cos(theta)*v,  math.sin(theta)*v]

    def get_vxvy_from_agent(self,agent: Agent):

        beta = np.arctan(1 / 2 * np.tan(agent.velocity[1]))
        # print('pre pose ',agent.position[0]+agent.velocity[0] *np.cos(agent.theta + beta)*0.02,agent.position[1]+agent.velocity[1] *np.cos(agent.theta + beta)*0.02)
        beta = 0

        return agent.velocity[0] * np.cos(agent.theta + beta), agent.velocity[0] * np.sin(agent.theta + beta)

    def orca(self,agent, colliding_agents, t, dt, limit=None, method='orca'):
        """Compute ORCA solution for agent. NOTE: velocity must be _instantly_
        changed on tick *edge*, like first-order integration, otherwise the method
        undercompensates and you will still risk colliding."""
        # print('orca_src ',colliding_agents)
        if limit is None:
            limit = self.limit_range
        else:
            self.limit_range = limit
        lines = []
        for collider in colliding_agents:
            # print(collider)
            if method == "avo":
                dv, n = self.get_avoidance_velocity(agent, collider, t, dt)
            else:
                dv, n = self.get_car_orca_avoidance_velocity(agent, collider, t, dt, limit)

            line = Line(agent.velocity + dv, n)  # 这里本来应该是个个体都要有一半的避障责任（dv/2）
            lines.append(line)
            # print('collider ',collider.position,collider.velocity,' -- to line ',agent.velocity + dv / 2, n)
        lines.append(Line([0, limit[0] - agent.position[1]], [0, 1]))
        lines.append(Line([0, limit[1] - agent.position[1]], [0, -1]))

        # velo
        return self.speed_optimize(lines, agent, t, dt, limit), lines

    def get_avoidance_velocity(self,agent, collider, t, dt):
        """Get the smallest relative change in velocity between agent and collider
        that will get them onto the boundary of each other's velocity obstacle
        (VO), and thus avert collision."""

        # This is a summary of the explanation from the AVO paper.
        #
        # The set of all relative velocities that will cause a collision within
        # time tau is called the velocity obstacle (VO). If the relative velocity
        # is outside of the VO, no collision will happen for at least tau time.
        #
        # The VO for two moving disks is a circularly truncated triangle
        # (spherically truncated cone in 3D), with an imaginary apex at the
        # origin. It can be described by a union of disks:
        #
        # Define an open disk centered at p with radius r:
        # D(p, r) := {q | ||q - p|| < r}        (1)
        #
        # Two disks will collide at time t iff ||x + vt|| < r, where x is the
        # displacement, v is the relative velocity, and r is the sum of their
        # radii.
        #
        # Divide by t:  ||x/t + v|| < r/t,
        # Rearrange: ||v - (-x/t)|| < r/t.
        #
        # By (1), this is a disk D(-x/t, r/t), and it is the set of all velocities
        # that will cause a collision at time t.
        #
        # We can now define the VO for time tau as the union of all such disks
        # D(-x/t, r/t) for 0 < t <= tau.
        #
        # Note that the displacement and radius scale _inversely_ proportionally
        # to t, generating a line of disks of increasing radius starting at -x/t.
        # This is what gives the VO its cone shape. The _closest_ velocity disk is
        # at D(-x/tau, r/tau), and this truncates the VO.

        x = -(agent.position - collider.position)
        v = agent.velocity - collider.velocity
        r = agent.radius + collider.radius

        x_len_sq = self._norm_sq(x)

        if x_len_sq >= r * r:
            # We need to decide whether to project onto the disk truncating the VO
            # or onto the sides.
            #
            # The center of the truncating disk doesn't mark the line between
            # projecting onto the sides or the disk, since the sides are not
            # parallel to the displacement. We need to bring it a bit closer. How
            # much closer can be worked out by similar triangles. It works out
            # that the new point is at x/t cos(theta)^2, where theta is the angle
            # of the aperture (so sin^2(theta) = (r/||x||)^2).
            adjusted_center = x / t * (1 - (r * r) / x_len_sq)

            if dot(v - adjusted_center, adjusted_center) < 0:
                # v lies in the front part of the cone
                # print("front")
                # print("front", adjusted_center, x_len_sq, r, x, t)
                w = v - x / t
                u = self._normalized(w) * r / t - w
                n = self._normalized(w)
            else:  # v lies in the rest of the cone
                # print("sides")
                # Rotate x in the direction of v, to make it a side of the cone.
                # Then project v onto that, and calculate the difference.
                leg_len = sqrt(x_len_sq - r * r)
                # The sign of the sine determines which side to project on.
                sine = copysign(r, det((v, x)))
                rot = array(
                    ((leg_len, sine),
                     (-sine, leg_len)))
                rotated_x = rot.dot(x) / x_len_sq
                n = self._perp(rotated_x)
                if sine < 0:
                    # Need to flip the direction of the line to make the
                    # half-plane point out of the cone.
                    n = -n
                # print("rotated_x=%s" % rotated_x)
                u = rotated_x * dot(v, rotated_x) - v
                # print("u=%s" % u)
        else:
            # We're already intersecting. Pick the closest velocity to our
            # velocity that will get us out of the collision within the next
            # timestep.
            # print("intersecting")
            # w = -v - x/dt
            # u = (normalized(w) * r/dt - w)
            # n =( normalized(w))
            # if 1:
            w = -v - x / dt
            u = w
            n = self._normalized(-x)
        return u, n

    def get_car_orca_avoidance_velocity(self,agent:Agent, collider, t, dt, limit):

        x = -(agent.position - collider.position)
        v = agent.velocity - collider.velocity
        r = agent.radius + collider.radius

        x_len_sq = self._norm_sq(x)

        if x_len_sq >= r * r:
            collider_aviliable_speed_set = self.get_car_aviliable_speed(collider, t, limit)
            unino_agent_collide_speed_set = self.get_agent_collide_set(agent, collider, t)
            agent_collide_set = Minkowski.Minkowski_sum(collider_aviliable_speed_set, unino_agent_collide_speed_set)
            u, n = self.get_dv_n_from_tubianxing(agent.velocity, agent_collide_set)


        else:
            # We're already intersecting. Pick the closest velocity to our
            # velocity that will get us out of the collision within the next
            # timestep.
            # print("intersecting")
            # w = -v - x/dt
            # u = (normalized(w) * r/dt - w)
            # n =( normalized(w))
            # if 1:
            w = -v - x / dt
            u = w
            n = self._normalized(-x)
        return u, n

    def speed_optimize(self,lines, agent, t, dt, limit):
        contorl_arr = []
        reward_arr = []
        control_v_arr = []
        init_vx, init_vy = self.get_vxvy_from_agent(agent)

        control_v_arr = self.get_car_posiable_speed_car(agent)
        control_v_arr = self.limit_v_choose(control_v_arr, agent, dt, limit)

        for i in range(len(control_v_arr)):
            tmp_vx = control_v_arr[i][0]
            tmp_vy = control_v_arr[i][1]
            tmp_reward = self.reward_speed(agent, [tmp_vx, tmp_vy], t, dt, lines)
            # print('reward ',tmp_reward,' action  ',[tmp_vx,tmp_vy],i,j,' ori ',init_vx,init_vy)
            contorl_arr.append([tmp_vx, tmp_vy])
            reward_arr.append(tmp_reward)

        final_control =self.speed_choose(agent, reward_arr, contorl_arr, t)
        # print('final reward ',reward_arr[contorl_arr.index(final_control)],' derta v control ',final_control[0]-init_vx,final_control[1]-init_vy)
        # print('action ',final_control,' pre ',control_v_create(agent, final_control, t))
        return final_control

    def speed_choose(self,agent: Agent, rewards, actions, t):
        min_coll = 9999
        for i in range(len(rewards)):
            if rewards[i][0] < min_coll:
                min_coll = rewards[i][0]
        w_speed = 0.3
        w_orca = 0.7
        w_rank = 0

        min_reward = 99999
        min_action = []
        record_reward = []
        if min_reward == -1:
            for i in range(len(rewards)):
                if rewards[i][0] == min_coll:
                    control_v = actions[i]
                    tmp_reward = rewards[i][1] + math.hypot(agent.pref_velocity[0] - control_v[0],
                                                            agent.pref_velocity[1] - control_v[1])
                    if tmp_reward < min_reward:
                        min_reward = tmp_reward
                        min_action = actions[i]
        else:
            for i in range(len(rewards)):
                control_v = actions[i]
                re_speed = math.hypot(agent.pref_velocity[0] - control_v[0], agent.pref_velocity[1] - control_v[1])
                re_orca = rewards[i][1]
                re_rank = rewards[i][0]
                tmp_reward = w_speed * re_speed + w_orca * re_orca + w_rank * re_rank
                # print(actions[i],'--',w_speed*re_speed,w_orca*re_orca,w_rank*re_rank)
                if tmp_reward < min_reward:
                    min_reward = tmp_reward
                    record_reward = [re_speed, re_orca, re_rank]
                    min_action = actions[i]

        # print('speed reward ',min_reward,record_reward ,' action ',min_action)
        # print("control choose")
        return min_action

    def reward_speed(self,agent, control_v, t, dt, lines):
        reward = self.reward_point_to_lines(control_v, lines)
        reward[1] += self.reward_limit(agent, control_v)
        return reward

    def reward_point_to_lines(self,point, lines):
        reward = 0
        count = 0
        for line in lines:
            re = self.reward_point_to_line(point, line)
            # print(re)
            if re > self.save_dis * self.save_reward:
                count += 1
            reward += re
        return [count, reward]

    def reward_point_to_line(self,point, line):
        x = np.array(point - line.point)
        y = np.array(line.direction)
        Lx = np.sqrt(x.dot(x))
        Ly = np.sqrt(y.dot(y))
        v3 = np.array(x - y)
        Lc = np.sqrt(v3.dot(v3))
        cos_angle = (Lx * Lx + Ly * Ly - Lc * Lc) / (2 * Ly * Lx)
        # print(cos_angle*180/math.pi)
        # theta = math.acos(cos_angle)
        if cos_angle > 1:
            cos_angle = 1
        if cos_angle < -1:
            cos_angle = -1
        theta = math.atan2(math.sqrt(1 - cos_angle * cos_angle), cos_angle)
        # print(theta/math.pi*180)
        if theta <= math.pi / 2:
            if line.direction[1] != 0:
                k = -line.direction[0] / line.direction[1]
                # print('k ',k,' pose ',x,' dir ',y,' ori ',point,' line point ',line.point)
                distant = abs(k * x[0] - x[1]) / math.sqrt(k * k + 1)
            else:
                distant = abs(x[1])

            if distant >= self.save_dis:
                return 0
            else:
                return self.save_reward * (self.save_dis - distant)

        else:
            if line.direction[1] != 0:
                k = -line.direction[0] / line.direction[1]
                # print('k ',k,' pose ',x,' dir ',y,' ori ',point,' line point ',line.point)
                distant = abs(k * x[0] - x[1]) / math.sqrt(k * k + 1)
            else:
                distant = abs(x[1])
            return distant + self.save_reward * self.save_dis

    def reward_limit(self,agent, control_v):
        if agent.position[1] + control_v[1] < -1 or agent.position[1] + control_v[1] > 13:
            return 999
        return 0

    def limit_v_choose(self,v_arr, agent, dt, limit):
        for v in v_arr:
            dy = agent.position[1] + v[1] * dt * 3
            if dy < limit[0] or dy > limit[1]:
                v_arr.remove(v)
        return v_arr

    def get_car_posiable_speed_car(self,agent: Agent):
        ans = []
        # init_vx,init_vy=get_vxvy_from_agent(agent)
        init_vx = agent.velocity[0]
        init_vy = agent.velocity[1]

        derta_vx = 0.3
        acc_range_number = 10

        max_theta = math.pi / 4
        steer_range_number = 10

        for i in range(-acc_range_number, acc_range_number + 1):
            for j in range(-steer_range_number, steer_range_number + 1):
                tmp_theta = j / steer_range_number * max_theta
                ds = i * derta_vx / math.cos(tmp_theta)

                tmp_vx = init_vx + ds * math.cos(agent.theta - tmp_theta)
                tmp_vy = init_vy + ds * math.sin(agent.theta - tmp_theta)

                # print('agent ',init_vx,init_vy,agent.theta,' derta  ',i*derta_vx,tmp_theta*180/math.pi,' tmp  ',[tmp_vx,tmp_vy])
                ans.append([tmp_vx, tmp_vy])

        # for i in ans:
        #     print(i)
        # print(ans)
        return ans

    def get_car_aviliable_speed(self, collider: Agent, t, limit):
        houxuan = collider.radius * 2 / 4
        up_range = limit[1]
        down_range = limit[0]
        min_turning_radiu = 15
        # print('collider.theta ',collider.theta)

        s_max = collider.velocity[0] * t + 1 / 2 * collider.max_speed * t * t
        speedmax_theta = s_max / min_turning_radiu

        # 求车辆朝着y值增加的方向转弯所能转过的最大角
        o_up = [collider.position[0] - min_turning_radiu * math.sin(collider.theta),
                collider.position[1] + min_turning_radiu * math.cos(collider.theta)]
        if o_up[1] > up_range:
            o_up_turn_cos_theta = (o_up[1] - up_range) / min_turning_radiu
            if o_up_turn_cos_theta > 1:
                o_up_turn_cos_theta = 1
            if o_up_turn_cos_theta < -1:
                o_up_turn_cos_theta = -1
            o_up_theta = math.acos(o_up_turn_cos_theta) - collider.theta
        else:
            o_up_turn_cos_theta = (up_range - o_up[1]) / min_turning_radiu
            if o_up_turn_cos_theta > 1:
                o_up_turn_cos_theta = 1
            if o_up_turn_cos_theta < -1:
                o_up_turn_cos_theta = -1
            o_up_theta = math.pi - math.acos(o_up_turn_cos_theta) - collider.theta

        up_theta_limit = min(o_up_theta, speedmax_theta)

        # 求车辆朝着y值ecreace的方向转弯所能转过的最大角
        o_down = [collider.position[0] + min_turning_radiu * math.sin(collider.theta),
                  collider.position[1] - min_turning_radiu * math.cos(collider.theta)]
        if o_down[1] < down_range:
            o_down_turn_cos_theta = (down_range - o_down[1]) / min_turning_radiu
            if o_down_turn_cos_theta > 1:
                o_down_turn_cos_theta = 1
            if o_down_turn_cos_theta < -1:
                o_down_turn_cos_theta = -1
            o_down_theta = math.acos(o_down_turn_cos_theta) + collider.theta
        else:
            o_down_turn_cos_theta = -(down_range - o_down[1]) / min_turning_radiu
            if o_down_turn_cos_theta > 1:
                o_down_turn_cos_theta = 1
            if o_down_turn_cos_theta < -1:
                o_down_turn_cos_theta = -1
            o_down_theta = math.pi - math.acos(o_down_turn_cos_theta) + collider.theta
        down_theta_limit = min(o_down_theta, speedmax_theta)

        # zuo you bian ti xing de chang du
        if collider.velocity[0] - collider.max_speed * t > 0:
            down_l = [(collider.velocity[0] - collider.max_speed * t),
                      (collider.velocity[0] + collider.max_speed * t) / math.cos(down_theta_limit)]
            up_l = [(collider.velocity[0] - collider.max_speed * t),
                    (collider.velocity[0] + collider.max_speed * t) / math.cos(up_theta_limit)]
        else:
            up_l = [0, (collider.velocity[0] + collider.max_speed * t) / math.cos(up_theta_limit)]

            down_l = [0, (collider.velocity[0] + collider.max_speed * t) / math.cos(down_theta_limit)]

        v0 = [math.cos(collider.theta), math.sin(collider.theta)]

        v_up = np.array([v0[0] * math.cos(up_theta_limit) - v0[1] * math.sin(up_theta_limit),
                         v0[0] * math.sin(up_theta_limit) + v0[1] * math.cos(up_theta_limit)])
        v_down = np.array([v0[0] * math.cos(down_theta_limit) + v0[1] * math.sin(down_theta_limit),
                           -v0[0] * math.sin(down_theta_limit) + v0[1] * math.cos(down_theta_limit)])

        v_up = self._normalized(v_up)
        v_down = self._normalized(v_down)

        return [v_up * up_l[0], v_up * up_l[1], down_l[1] * v_down, down_l[0] * v_down]

    def get_agent_collide_set(self,agent, collider, t):
        max_dis = 100
        x = -(agent.position - collider.position)
        v = agent.velocity - collider.velocity
        r = agent.radius + collider.radius
        len_x = sqrt(self._norm_sq(x))

        sin_theta = r / len_x
        if sin_theta > 1:
            sin_theta = 1
        if sin_theta < -1:
            sin_theta = -1
        derta_theta = math.asin(sin_theta)
        cos_theta = math.cos(derta_theta)
        l = (len_x - r) / t / cos_theta
        norm_x = self._normalized(x)
        v1 = np.array([norm_x[0] * cos_theta - norm_x[1] * sin_theta, norm_x[0] * sin_theta + norm_x[1] * cos_theta])
        v2 = np.array([norm_x[0] * cos_theta + norm_x[1] * sin_theta, -norm_x[0] * sin_theta + norm_x[1] * cos_theta])

        return [v1 * l, v1 * max_dis, v2 * max_dis, v2 * l]

    def get_dv_n_from_tubianxing(self,point, points_list):
        proj_point, seg = tubianxing.get_min_point_in_tubianxing(point, points_list)
        in_tubianxing = tubianxing.IsPointInConvexPolygon(points_list, point)
        dv = np.array([proj_point[0] - point[0], proj_point[1] - point[1]])
        n = self._normalized(self._perp(np.array([seg[1][0] - seg[0][0], seg[1][1] - seg[0][1]])))
        theta = abs(math.atan2(dv[1], dv[0]) - math.atan2(n[1], n[0]))
        if in_tubianxing == True and theta > math.pi / 2:
            n = -n

        if in_tubianxing == False and theta < math.pi / 2:
            n = -n

        return dv, n

    def get_speed_reward(self,lines, agent, v, t, dt):
        tmp_reward = self.reward_speed(agent, v, t, dt, lines)
        w_speed = 0.35
        w_orca = 0.65
        w_rank = 0
        control_v = v
        re_speed = math.hypot(agent.pref_velocity[0] - control_v[0], agent.pref_velocity[1] - control_v[1])
        re_orca = tmp_reward[1]
        re_rank = tmp_reward[0]
        reward = w_speed * re_speed + w_orca * re_orca + w_rank * re_rank
        return reward

    def draw_orca_collider(self,agent, collider, t, dt,limit):
        from orca_src import draw_picture

        x = -(agent.position - collider.position)
        r = agent.radius + collider.radius

        x_len_sq = self._norm_sq(x)

        if x_len_sq < r * r:
            return None,None
        print('agent ',agent.position,agent.velocity,' collider',collider.position,collider.velocity)
        collider_aviliable_speed_set= self.get_car_aviliable_speed(collider, t, limit)
        unino_agent_collide_speed_set= self.get_agent_collide_set(agent, collider, t)
        agent_collide_set= Minkowski_sum(collider_aviliable_speed_set, unino_agent_collide_speed_set)
        u,n= self.get_dv_n_from_tubianxing(agent.velocity, agent_collide_set)

        print(collider_aviliable_speed_set)
        print(unino_agent_collide_speed_set)


        plt.figure(num='orca_src')
        plt.clf()
        # 画图
        x_major_locator = MultipleLocator(10)
        y_major_locator = MultipleLocator(10)
        ax = plt.gca()# ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)


        for i in range(len(collider_aviliable_speed_set)):
            collider_aviliable_speed_set[i]+=x

        plt.scatter(0,0,color='red')
        plt.scatter(x[0],x[1],color='blue')

        plt.plot([0,agent.velocity[0]],[0,agent.velocity[1]],color='black')
        plt.plot([agent.velocity[0],agent.velocity[0]+u[0]],[agent.velocity[1],agent.velocity[1]+u[1]],color='red')

        draw_picture.draw_tubianxing(collider_aviliable_speed_set,color='blue')
        draw_picture.draw_tubianxing(unino_agent_collide_speed_set,color='red')
        draw_picture.draw_tubianxing(agent_collide_set,color='black')
        plt.show()


        return u,n

    def draw(self, my_agent:Agent, agents, lines,v_opt):
        #设置显示范围
        show_range=30
        ylimit=[-5,18]
        show_rate=12

        plt.figure(num='route', figsize=(show_range*6/show_rate,(ylimit[1]-ylimit[0])/show_rate) )
        plt.clf()
        # 画图
        x_major_locator = MultipleLocator(10)
        y_major_locator = MultipleLocator(10)
        ax = plt.gca()# ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)
        plt.xlim(my_agent.position[0] - show_range * 3, my_agent.position[0] + show_range * 3)
        plt.ylim(ylimit)
        plt.gca().invert_yaxis()



        #绘制车道线
        for i in range(5):
            plt.plot([my_agent.position[0] - show_range * 3, my_agent.position[0] + show_range * 3], [i * 4 - 2, i * 4 - 2], color='grey')

        #绘制障碍物
        for ag in agents:
            self.draw_circle(ag)

        #绘制半平面
        for line in lines:
            # print('before ',line)
            self.draw_half_line(my_agent,line)
        # print('velo ',my_agent.velocity)
        # 绘制控制车辆
        self.draw_circle(my_agent, colr='black')

        # 绘制控制车辆各种速度
        # plt.plot([my_agent.position[0],my_agent.position[0]+my_agent.pref_velocity[0]],[my_agent.position[1],my_agent.position[1]+my_agent.pref_velocity[1]], color='red')
        plt.plot([my_agent.position[0], my_agent.position[0] + v_opt[0]],
                 [my_agent.position[1], my_agent.position[1] + v_opt[1]], color='blue')

        # print(my_agent.velocity[1],np.arctan(1 / 2 * np.tan(my_agent.velocity[1])))
        # beta = np.arctan(1 / 2 * np.tan(my_agent.velocity[1]))
        # print('pre pose ', my_agent.position[0] + my_agent.velocity[0] * np.cos(my_agent.theta + beta) * 0.02,
        #       my_agent.position[1] + my_agent.velocity[1] * np.cos(my_agent.theta + beta) * 0.02)

        # plt.show()
        plt.pause(0.01)
        # input()

    def draw_speed_reward(self,orca_v,agent,lines):
        import matplotlib.pyplot as plt
        from matplotlib.ticker import LinearLocator
        import numpy as np

        x_range=30
        x=[]
        y=[]
        z=[]
        max_reward=10
        for i in range(int(orca_v[0])-x_range,int(orca_v[0])+x_range):
            x_arr=[]
            y_arr=[]
            z_arr=[]
            for j in range(-30,150,1):
                # k=i+j
                k=-self.get_speed_reward(lines, agent, [i, j / 10], self.tau, self.dt)
                if k<-max_reward:
                    k=-max_reward
                x_arr.append(i)
                y_arr.append(j)
                z_arr.append(k)
            x.append(x_arr)
            y.append(y_arr)
            z.append(z_arr)

        z=np.array(z)
        x=np.array(x)
        y=np.array(y)



        fig = plt.figure()
        ax = fig.gca(projection='3d')
        surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='rainbow')

        ax.set_xlabel('road')
        ax.set_ylabel('lanes')
        ax.set_zlabel('reward')
        ax.set_zlim(-10, 2)
        ax.set_ylim(-3, 15)
        ax.zaxis.set_major_locator(LinearLocator(10))

        fig.colorbar(surf, shrink=1, aspect=10)


        # Add a color bar which maps values to colors.

        plt.show()

        return

    def draw_circle(self, agent, colr="r"):
        r=agent.radius
        x=agent.position[0]
        y=agent.position[1]
        # 点的横坐标为a
        a = np.arange(x - r, x + r, 0.001)
        # 点的纵坐标为b
        b = np.sqrt(np.power(r, 2) - np.power((a - x), 2))
        plt.plot(a, b+ y, color=colr, linestyle='-')
        plt.plot(a, -b+ y, color=colr, linestyle='-')
        plt.scatter(x, y, c='b', marker='o')
        plt.plot([x,x+agent.velocity[0]/5],[y,y+agent.velocity[1]/5], color=colr, linestyle='-')

    def draw_half_line(self,agent:Agent,line:Line):
        len=50
        init_pose=agent.position+line.point
        # print('after ',init_pose,agent.position,line.point)
        plt.plot([init_pose[0],init_pose[0]+line.direction[0]],[init_pose[1],init_pose[1]+line.direction[1]],color='green')
        pre=self.__perp(line.direction)
        plt.plot([init_pose[0],init_pose[0]+pre[0]*len],[init_pose[1],init_pose[1]+pre[1]*len],color='green')
        plt.plot([init_pose[0],init_pose[0]-pre[0]*len],[init_pose[1],init_pose[1]-pre[1]*len],color='green')

        # plt.scatter(init_pose[0],init_pose[1],color='green')

