import gym
import highway_env

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
while not done:
    # action =env.action_space.sample()
    action=(0.8,0)
    obs, reward, done, info = env.step(action)
    print('---------------------------------------------')
    # print('action ',action, type(action))
    # print('obs ',obs)
    for obj in obs:
        print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )
    # print('info ',info)
    env.render()
    input()