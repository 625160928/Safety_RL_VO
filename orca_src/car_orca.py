import math
import numpy as np

from env.highway_sim_env import HighwaySimulation


from orca_src.orca import Orca
from orca_src.class_agent import Agent

class CarOrca(Orca):
    def __init__(self,sim_env=None,method='avo'):
        Orca.__init__(self,method='avo')
        self.fresh_speed=False
        self.env=sim_env
        self.done = False

    def get_action(self, obs, method=None):
        return self.get_action_from_obs(obs, method)

    #速度控制器
    def PControl(self, target, current):
        return self.Speed_Kp * (target - current)


    def get_agent_from_obj(self, obj):
        position=(obj[1], obj[2])
        velocity = (obj[3], obj[4])

        agent=Agent(position,velocity, self.car_radiu,  self.acc, (obj[3], obj[4]),theta=math.atan2(obj[6],obj[5]))
        # if math.atan2(obj[6],obj[5])!=0:
        #     print('obj ', obj, '  theta ', math.atan2(obj[6], obj[5]))
        vx,vy= self.get_vxvy_from_agent(agent)
        agent.velocity=np.array((vx,vy))
        return agent


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

    def get_action_from_agents(self,agents, method):

        # 计算orca避障速度
        new_vels, all_line = self.orca(agents[0], agents[1:], self.tau, self.dt
                                  , limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain]
                                  , method=method)

        # 将速度转换为动作指令
        new_vels[0] = new_vels[0]+1
        new_vels[1] = new_vels[1] + 0.3
        action = self.change_vxvy_to_action(agents[0], new_vels)
        return new_vels,action

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


    # et orca_src action from orca_src
    def get_action_from_obs(self, obs, method=None):

        # 将obs的数据格式转换成orca使用的agents类型
        agents = []
        for obj in obs:
            pd = False
            for i in obj:
                if i != 0:
                    pd = True
            if pd:
                agents.append(self.get_agent_from_obj(obj))
                # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )

        # 设置目标速度，换道决策
        agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                              agents[0].position[0] + 80,
                                                              self.lane * self.lane_length,
                                                              self.prev)
        # 计算orca避障速度
        new_vels,action=self.get_action_from_agents(agents,method)

        return action, new_vels


def main():
    #此处的运行实例看旧版本的highway
    pass


if __name__ == '__main__':

    main()