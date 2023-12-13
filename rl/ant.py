import gymnasium as gym
import sys, os
sys.path.append(os.path.abspath(os.getcwd()))
import rl.envs
from stable_baselines3 import A2C, PPO

env = gym.make("AntRL", render_mode="human")
# env = gym.make("AntRL", render_mode=None)
env.action_space.seed(42)

train = False
# train = True

device = 'cuda'
# device = 'cpu'

runs_folder = 'rl/runs/ant'
os.makedirs(runs_folder, exist_ok=True)
checkpoint_filename = os.path.sep.join([runs_folder, 'ant_save.zip'])

# model = A2C("MlpPolicy", env, verbose=1, device=device)
model = PPO("MlpPolicy", env, verbose=1, device=device)
if os.path.exists(checkpoint_filename):
    model = model.load(checkpoint_filename, env, device=device)
if train:
    for i in range(0,50):
        model.learn(total_timesteps=10_000, progress_bar=True)
        model.save(checkpoint_filename)

observation, info = env.reset()
vec_env = model.get_env()


for _ in range(10000):
    action, _state = model.predict(observation, deterministic=True)
    # action = env.action_space.sample()  # agent policy that uses the observation and info
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()