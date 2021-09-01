import math

import numpy as np
import gym
import highway_env
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from halfplaneintersect import Line
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp

from controller import pid_lateral_controller_angle
# from controller import pid_longitudinal_controller

class HighWayOrca():
    def __init__(self):
        self.dt=0.05
        self.car_steer_limit=math.pi/3
        self.later_pid=pid_lateral_controller_angle.PIDLateralController(L=2.5, dt=self.dt, car_steer_limit=self.car_steer_limit, K_P=0.2, K_D=0.0, K_I=0.0)
        self.env = gym.make("highway-v0")
        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)
        config = {
            'vehicles_count':2,
            'simulation_frequency': 150,
            'vehicles_density': 1,
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 15,
                "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
                "features_range": {
                    "x": [-100, 100],
                    "y": [-100, 100],
                    "vx": [-20, 20],
                    "vy": [-20, 20]
                },
                "absolute": True,
                "normalize": False,
                "order": "sorted"
            },
            "action": {
                "type": "ContinuousAction"
            }
        }

        self.env.configure(config)
        self.env.reset()
        self.done = False
        self.acc = 4
        self.tau = 10
        self.prev = 10
        self.lane=1
        self.lane_length=4
        # 速度P控制器系数 1
        self.Speed_Kp = 0.1



    #速度控制器
    def __PControl(self, target, current):
        return self.Speed_Kp * (target - current)

    def set_pref_v(self,current_x,current_y,goal_x,goal_y,v):
        theta = math.atan2(goal_y - current_y, goal_x - current_x)
        return  [math.cos(theta)*v,  math.sin(theta)*v]

    def change_vxvy_to_action(self,agent:Agent,control_v):
        l=math.hypot(control_v[0],control_v[1])
        # print(agent.position[0],agent.position[1],agent.theta*180/math.pi)
        control_theta=math.atan2(control_v[1],control_v[0])
        goal_x=math.cos(control_theta)*l
        goal_y=math.sin(control_theta)*l
        now_v=math.hypot(agent.velocity[0],agent.velocity[1])
        steer=self.later_pid.run_step(0, 0, agent.theta, goal_x, goal_y, control_theta, now_v)
        # return steer
        ai= self.__PControl(now_v, l)
        action=[ai,-steer]
        print('pose ',agent.position,agent.theta,'pre-v',agent.pref_velocity,'orca-v',control_v,'action ',action)
        return action
        # return [0.5,0]

    def run(self):

        action = (0, 0)
        while not self.done:
            # action =env.action_space.sample()
            obs, reward, self.done, info = self.env.step(action)
            print('---------------------------------------------')
            # print('action ',action, type(action))
            # print('obs ',obs)
            if self.done:
                break

            agents = []
            for obj in obs:
                pd=False
                for i in obj:
                    if i !=0:
                        pd=True
                if pd:
                    agents.append(Agent((obj[1], obj[2]), (obj[3], obj[4]), 2., self.tau * self.acc, (obj[3], obj[4]),theta=math.atan2(obj[6],obj[5])))
                    # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )
            # print('info ',info)

            # 设置目标速度，换道决策
            agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                 agents[0].position[0] + 80, self.lane*self.lane_length, self.prev)
            # print('pose ',agents[0].position,' theta ',theta*180/math.pi,math.sin(theta))

            # 计算orca避障速度
            new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt,limit=[-2,14])
            # print('line ',all_line)
            # 将速度转换为动作指令
            action = self.change_vxvy_to_action(agents[0], new_vels)
            # print('old v',agents[0].velocity,'new v ',new_vels,' action ',action,' pre  ',agents[0].pref_velocity,theta*180/math.pi)
            # print(action, agents[0].position, agents[0].pref_velocity, new_vels)

            self.env.render()
            # input()

            self.draw(agents[0], agents[1:],all_line,new_vels)

    def draw(self, my_agent:Agent, agents, lines,v_opt):
        plt.figure(num='route', dpi=70 )
        plt.clf()
        #设置显示范围
        show_range=30
        ylimit=[-5,18]
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


        #绘制控制车辆
        self.draw_circle(my_agent, colr='black')

        #绘制控制车辆各种速度
        # plt.plot([my_agent.position[0],my_agent.position[0]+my_agent.pref_velocity[0]],[my_agent.position[1],my_agent.position[1]+my_agent.pref_velocity[1]], color='red')
        plt.plot([my_agent.position[0], my_agent.position[0] + v_opt[0]],
                 [my_agent.position[1], my_agent.position[1] +v_opt[1]], color='blue')

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

        # print(len(lines),len(agents))

        # plt.show()
        plt.pause(0.1)
        input()

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
        init_pose=agent.position+line.point
        # print('after ',init_pose,agent.position,line.point)
        plt.plot([init_pose[0],init_pose[0]+line.direction[0]],[init_pose[1],init_pose[1]+line.direction[1]],color='green')
        plt.scatter(init_pose[0],init_pose[1],color='green')

def main():
   new_highway_orca=HighWayOrca()
   new_highway_orca.run()

if __name__ == '__main__':

    main()