import gymnasium as gym
import sys, os
sys.path.append(os.path.abspath(os.getcwd()))
import rl.envs
# env = gym.make("Ant-v4", render_mode="human")
env = gym.make("AntRL", render_mode="human")
observation, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()