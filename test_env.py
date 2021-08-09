import gym
import matlipot_env
import numpy as np
from npc_car.car_model import simulate_car_model

player_model= simulate_car_model.create_car_example()

env = gym.make('lane_change-v0',model=player_model, init_lane=0, empty_lane=1, seed=10, render_keep=False)

for i in range(100):

    action1 = np.array([[50],[0]])
    observation, reward, done, info = env.step(action1)
    
    print(reward)

    if done == True:
        env.reset()
    
    env.render()