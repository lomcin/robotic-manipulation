from gymnasium.envs.registration import make, register, registry, spec

register(
    id="AntRL",
    entry_point="rl.envs.mujoco:AntRLEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)