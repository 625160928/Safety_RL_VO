import sys
sys.path.append("E:\\robot_rl\lane_change\Safety_RL_VO")
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
# from matlipot_env import OneLaneEmptyEnv
from npc_car.npc_car import NPCCar
from npc_car.car_model import simulate_car_model
from matlipot_env.one_lane_empty_env import OneLaneEmptyEnv


class one_lane_env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, model, init_lane=0, empty_lane=1, seed=10, render_keep=False):
        
        # **kwargs:
        #     start_mode: the mode of the start and goal point arrangement
        #           0: face to face
        #           1: forward 
        #           2: random     
        # sg_kwargs:
        #      interval: interval between start and goal point
        #      upper, lower: range of the start and goal point
        # con_kwargs:
        #      robot_mode: the mode of the robot, diff or omni
        #      step_time: default time is 0.1
        #      goal_threshold: default 0.2

        self.model = model
        self.init_lane = init_lane
        self.empty_lane = empty_lane
        self.ran_seed = seed
        self.render_keep = render_keep

        self.lane_env = OneLaneEmptyEnv(model=model, init_lane=init_lane, empty_lane=empty_lane, seed=self.ran_seed)
        
        dim = 5
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
    

    def step(self, action):

        v = float(action[0, 0])
        w = float(action[1,0])
        self.lane_env.update(v,w)

        observation = self.lane_env.obs()
        # reward = self.multi_robot.cal_reward(mode)
        reward = 1
        done = self.lane_env.check_done()
    
        info = None

        return observation, reward, done, info

        
    def reset(self):
        
        self.lane_env.reset()
        observation = self.lane_env.observation()

        return observation

    def render(self, mode = 'human'):
        self.lane_env.draw_env(keep=self.render_keep)


if __name__ == '__main__':
    player_model= simulate_car_model.create_car_example()
    env = one_lane_env(model=player_model, init_lane=0, empty_lane=1, seed=10, render_keep=False)
    action1 = np.array([[50],[0]])
    observation, reward, done, info = env.step(action1)
    print(observation)
    print(reward)
    print(done)
    action2 = np.array([[20],[1]])
    observation, reward, done, info = env.step(action2)
    print(observation)
    print(reward)
    print(done)