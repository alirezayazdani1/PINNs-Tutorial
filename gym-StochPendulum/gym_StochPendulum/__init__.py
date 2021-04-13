from gym.envs.registration import register

register(
    id='StochPendulum-v0',
    entry_point='gym_StochPendulum.envs:StochPendulumEnv',
    max_episode_steps=200,
)
