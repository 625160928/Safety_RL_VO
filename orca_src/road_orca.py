import math
import numpy as np

from orca_src.orca import Orca
from orca_src.class_agent import Agent

from controller import pid_lateral_controller_angle

"""
【类名】：CarOrca
【备注】：
原生的orca使用的控制输出是vx,vy
在进行封装继承后控制输出变成了油门转向
输入也添加了从highway输入的obs转换成orca使用的agent类的方法

纵向控制器使用的是pc控制器，其实就是线性加权
横向控制器目前用的是pid，后续改成其他更好的方案

【使用方法】



"""
class RoadOrca(Orca):
    #目前输入的sim_env是一整个环境，但实际上只可以用其中的参数
    #没对环境做保护，所以尽量不要调用env的render，step等方法
    def __init__(self,config=None,method='orca'):
        Orca.__init__(self,config=config,method=method)
        self.done = False
        self.Speed_Kp = config['Speed_Kp']
        self.edge_remain=config['edge_remain']
        self.car_radiu=config['car_radiu']
        self.lane=config['lane']
        self.lane_length=config['lane_length']
        self.acc=config['acc']

        self.later_pid=pid_lateral_controller_angle.PIDLateralController(L=2.5, dt=self.dt, car_steer_limit=self.car_steer_limit, K_P=4, K_D=0.2, K_I=0)

    #为highway，切换逻辑提供的输出反馈
    #输入的method指使用avo还是orca获取控制，只有【avo，orca】i啷个选择
    def get_action(self, obs, method=None,map=None):
        return self._get_action_from_obs(obs, method,map)

    #速度控制器
    #输入为目标速度与当前速度，输出为油门大小
    def _PControl(self, target, current):
        return self.Speed_Kp * (target - current)

    #从单个obj观察结果获取一个agent
    def _get_agent_from_obj(self, obj):
        position=(obj[1], obj[2])
        velocity = (obj[3], obj[4])

        agent=Agent(position,velocity, self.car_radiu,  self.acc, (obj[3], obj[4]),theta=math.atan2(obj[6],obj[5]))
        # if math.atan2(obj[6],obj[5])!=0:
        #     print('obj ', obj, '  theta ', math.atan2(obj[6], obj[5]))
        vx= agent.velocity[0] * np.cos(agent.theta )
        vy=agent.velocity[0] * np.sin(agent.theta )
        agent.velocity=np.array((vx,vy))
        return agent

    #从obs里提取出agent 的list
    def _get_agents_from_obs(self, obs):
        agents=[]
        new_speed=0
        for obj in obs:
            pd=False
            new_speed+=obj[3]
            for i in obj:
                if i !=0:
                    pd=True
            if pd:
                agents.append(self._get_agent_from_obj(obj))

        return agents

    #将正常orca的输出【vx,vy】,转换成车辆使用的控制【油门，转角】
    def _change_vxvy_to_action(self, agent:Agent, control_v):
        l=control_v[0]
        # print(agent.position[0],agent.position[1],agent.theta*180/math.pi)
        control_theta=math.atan2(control_v[1],control_v[0])
        goal_x=math.cos(control_theta)*l
        goal_y=math.sin(control_theta)*l

        now_v=agent.velocity[0]

        steer=self.later_pid.run_step(0, 0, agent.theta, goal_x, goal_y, control_theta, now_v)

        ai= self._PControl(l, now_v)

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

    #在orca的基础上进行封装，通过agent 的list 获取油门方向盘角度的控制（也有可能是前轮转角）
    def _get_action_from_agents(self, agents, method,grid_map=None):

        # 计算orca避障速度
        new_vels, all_line = self.orca(agents[0], agents[1:], self.tau, self.dt
                                  , method=method,grid_map=grid_map)

        # 将速度转换为动作指令
        new_vels[0] = new_vels[0]+1
        new_vels[1] = new_vels[1] + 0.3
        action = self._change_vxvy_to_action(agents[0], new_vels)
        return new_vels,action

    """
    输入是highway仿真环境的obs，控制方法method
    输入是车辆油门，方向盘角度
    """
    def _get_action_from_obs(self, obs, method=None,map=None):

        # 将obs的数据格式转换成orca使用的agents类型
        agents = []
        for obj in obs:
            pd = False
            for i in obj:
                if i != 0:
                    pd = True
            if pd:
                agents.append(self._get_agent_from_obj(obj))
                # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )

        # 设置目标速度，换道决策
        agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                              agents[0].position[0] + 80,
                                                              self.lane * self.lane_length,
                                                              self.prev)
        # 计算orca避障速度
        new_vels,action=self._get_action_from_agents(agents, method,map)

        return action, new_vels


def main():
    #此处的运行实例看旧版本的highway
    pass


if __name__ == '__main__':

    main()