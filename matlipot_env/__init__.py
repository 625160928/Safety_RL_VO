from gym.envs.registration import register

register(
    id='lane_change-v0',
    entry_point='matlipot_env.gym_wrapper:one_lane_env',
)