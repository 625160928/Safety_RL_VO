import math

import gym
import highway_env
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
env = gym.make("highway-v0")

config = {
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
        "normalize":False,
        "order": "sorted"
    },
    "action": {
        "type": "ContinuousAction"
    }
}

env.configure(config)
env.reset()
done = False
acc=4
tau=5
action=(0.8,0)
dt=0.05
prev=10
dir_y=8
while not done:
    # action =env.action_space.sample()
    obs, reward, done, info = env.step(action)
    print('---------------------------------------------')
    # print('action ',action, type(action))
    # print('obs ',obs)

    agents = []
    for obj in obs:
        agents.append(Agent((obj[1],obj[2]), (obj[3],obj[4]), 3., tau*acc, (obj[3],obj[4])))
        # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )
    # print('info ',info)

    #设置目标速度，换道决策
    theta=math.atan2(dir_y-agents[0].position[1],80)
    agents[0].pref_velocity=[prev*math.cos(theta),prev*math.sin(theta)]
    # print('pose ',agents[0].position,' theta ',theta*180/math.pi,math.sin(theta))

    #计算orca避障速度
    new_vels,all_line = orca(agents[0], agents[1:], tau, dt)

    #将速度转换为动作指令
    action=((new_vels[0]-agents[0].velocity[0])/40,-new_vels[1]/60)
    # print('old v',agents[0].velocity,'new v ',new_vels,' action ',action,' pre  ',agents[0].pref_velocity,theta*180/math.pi)

    env.render()