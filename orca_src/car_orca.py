import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from numpy import array, sqrt, copysign, dot


from orca_src import pyorca
from controller import pid_lateral_controller_angle
from orca_src.pyorca import Agent, orca,Line
from orca_src.Minkowski import Minkowski_sum


class Orca():
    def __init__(self,method='avo'):
        self.car_steer_limit=math.pi/3
        self.acc = 0.5
        self.tau = 2
        self.dt=0.05
        self.prev = 20
        self.lane=1
        self.lane_length=4

        # 速度P控制器系数 1
        self.Speed_Kp = 0.6
        self.car_radiu=3.2
        self.method=method
        self.edge_remain=0.3

        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)
        self.later_pid=pid_lateral_controller_angle.PIDLateralController(L=2.5, dt=self.dt, car_steer_limit=self.car_steer_limit, K_P=4, K_D=0.2, K_I=0)

    #速度控制器
    def PControl(self, target, current):
        return self.Speed_Kp * (target - current)


    def set_pref_v(self,current_x,current_y,goal_x,goal_y,v):
        theta = math.atan2(goal_y - current_y, goal_x - current_x)
        return  [math.cos(theta)*v,  math.sin(theta)*v]

    def change_vxvy_to_action(self,agent:Agent,control_v):
        l=control_v[0]
        # print(agent.position[0],agent.position[1],agent.theta*180/math.pi)
        control_theta=math.atan2(control_v[1],control_v[0])
        goal_x=math.cos(control_theta)*l
        goal_y=math.sin(control_theta)*l

        now_v=agent.velocity[0]

        steer=self.later_pid.run_step(0, 0, agent.theta, goal_x, goal_y, control_theta, now_v)

        ai= self.PControl( l,now_v)

        # if steer>math.pi/2:
        #     steer=math.pi/2
        # if steer < -math.pi / 2:
        #     steer = -math.pi / 2

        action=[ai,-steer]

        limit_control=7
        if control_v[0]<agent.velocity[0]-limit_control:
            print('emergy speed down')
            action[0]=-1
        if control_v[0]>agent.velocity[0]+limit_control:
            print('emergy speed up')
            action[0]=1
        # print('pose ', agent.position, agent.theta * 180 / math.pi, ' now_speed ', agent.velocity, 'pre-v',
        #       agent.pref_velocity, 'orca_src-v', control_v, 'action ', action)

        return action
        # return [0.5,0]

    def get_agent_from_obj(self, obj):
        position=(obj[1], obj[2])
        velocity = (obj[3], obj[4])

        agent=Agent(position,velocity, self.car_radiu,  self.acc, (obj[3], obj[4]),theta=math.atan2(obj[6],obj[5]))
        # if math.atan2(obj[6],obj[5])!=0:
        #     print('obj ', obj, '  theta ', math.atan2(obj[6], obj[5]))
        vx,vy= pyorca.get_vxvy_from_agent(agent)
        agent.velocity=np.array((vx,vy))
        return agent


    def draw_orca_collider(self,agent, collider, t, dt,limit):
        from orca_src import draw_picture

        x = -(agent.position - collider.position)
        r = agent.radius + collider.radius

        x_len_sq = pyorca.norm_sq(x)

        if x_len_sq < r * r:
            return None,None
        print('agent ',agent.position,agent.velocity,' collider',collider.position,collider.velocity)
        collider_aviliable_speed_set= pyorca.get_car_aciliable_speed(collider, t, limit)
        unino_agent_collide_speed_set= pyorca.get_agent_collide_set(agent, collider, t)
        agent_collide_set= Minkowski_sum(collider_aviliable_speed_set, unino_agent_collide_speed_set)
        u,n= pyorca.get_dv_n_from_tubianxing(agent.velocity, agent_collide_set)

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
                k=-pyorca.get_speed_reward(lines, agent, [i, j / 10], self.tau, self.dt)
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

    def __perp(self,a):
        return array((a[1], -a[0]))

    def draw_half_line(self,agent:Agent,line:Line):
        len=50
        init_pose=agent.position+line.point
        # print('after ',init_pose,agent.position,line.point)
        plt.plot([init_pose[0],init_pose[0]+line.direction[0]],[init_pose[1],init_pose[1]+line.direction[1]],color='green')
        pre=self.__perp(line.direction)
        plt.plot([init_pose[0],init_pose[0]+pre[0]*len],[init_pose[1],init_pose[1]+pre[1]*len],color='green')
        plt.plot([init_pose[0],init_pose[0]-pre[0]*len],[init_pose[1],init_pose[1]-pre[1]*len],color='green')

        # plt.scatter(init_pose[0],init_pose[1],color='green')

    def get_agents_from_obs(self,obs):
        agents=[]
        new_speed=0
        for obj in obs:
            pd=False
            new_speed+=obj[3]
            for i in obj:
                if i !=0:
                    pd=True
            if pd:
                agents.append(self.get_agent_from_obj(obj))

        return agents

    def get_action_from_agents(self,agents, method0):

        # 计算orca避障速度
        new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt
                                  , limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain]
                                  , method=method0)

        # 将速度转换为动作指令
        new_vels[0] = new_vels[0]+1
        new_vels[1] = new_vels[1] + 0.3
        action = self.change_vxvy_to_action(agents[0], new_vels)
        return action


    def run(self):

        action = (0, 0)

        while not self.done:
            # action =env.action_space.sample()
            obs, reward, self.done, info = self.env.step(action)

            if self.done:
                break

            agents = self.get_agents_from_obs(obs)


            # 设置目标速度，换道决策
            agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                 agents[0].position[0] + 80, self.lane*self.lane_length, self.prev)


            action=self.get_action_from_agents(agents)


            self.env.render()

