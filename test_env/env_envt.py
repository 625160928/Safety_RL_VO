import gym
import change_lane_simulate

env = gym.make('change_lane_env-v0')
env.step()
env.reset()

env2 = gym.make('change_lane_env_extend-v0')
env2.step()
env2.reset()