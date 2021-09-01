import math

import gym
import highway_env
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp

from controller import pid_lateral_controller_angle
# from controller import pid_longitudinal_controller

class HighWayOrca():
    def __init__(self):
        self.dt=0.05
        self.car_steer_limit=math.pi/3
        self.later_pid=pid_lateral_controller_angle.PIDLateralController(L=2.5, dt=self.dt, car_steer_limit=self.car_steer_limit, K_P=0.25, K_D=0.0, K_I=0.0)
        self.env = gym.make("highway-v0")
        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)
        config = {
            'vehicles_count':10,
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
        self.tau = 5
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

def main():
   new_highway_orca=HighWayOrca()
   new_highway_orca.run()

if __name__ == '__main__':

    main()