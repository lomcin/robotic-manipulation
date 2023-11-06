import time
import math
import pygame
pygame.init()

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

print(f'joysticks:{joysticks}')

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

m = mujoco.MjModel.from_xml_path('world/car4.xml')
d = mujoco.MjData(m)

max_accelerate = 10
acc_threshold = 0.1
cmd_acc_norm = 1.0-acc_threshold
turn_threshold = 0.1
cmd_turn_norm = 1.0-turn_threshold
light_active = True

joystick_acc_axis = 5
joystick_break_axis = 4

def update_state_from_joystick(state):
  global light_active
  for event in pygame.event.get():
    if event.type == pygame.JOYAXISMOTION:
      cmd_accelerate = joysticks[0].get_axis(joystick_acc_axis) > acc_threshold
      x_accelerate = (joysticks[0].get_axis(joystick_acc_axis) - acc_threshold)/(cmd_acc_norm)
      cmd_break = joysticks[0].get_axis(joystick_break_axis) > acc_threshold
      x_break = (joysticks[0].get_axis(joystick_break_axis) - acc_threshold)/cmd_acc_norm

      cmd_turn = np.abs(joysticks[0].get_axis(0)) > acc_threshold
      sign_y = np.sign(joysticks[0].get_axis(0))
      abs_y = np.abs(joysticks[0].get_axis(0))
      turn_val = sign_y*(abs_y - turn_threshold)/(cmd_turn_norm)

      if cmd_break and cmd_accelerate:
        state[0] = 0.0
      elif cmd_break:
        state[0] = -5*x_break*x_break
      elif cmd_accelerate:
        state[0] = max_accelerate*x_accelerate*x_accelerate
      else:
        state[0] = 0.0

      if cmd_turn:
        state[1] = -sign_y*max_accelerate*turn_val*turn_val
      else:
        state[1] = 0.0
    
    if event.type == pygame.JOYBUTTONDOWN:
      if joysticks[0].get_button(0):
        light_active = not light_active


with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  print(f'model:{dir(m)}')
  print(f'data:{dir(d)}')
  for i in range(0,m.njnt):
    print(f'jnt:{m.jnt(i)}')

  while viewer.is_running():
    step_start = time.time()

    mujoco.mj_step(m, d)

    tam = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_CTRL)
    state = np.zeros((tam,1),dtype=np.float64)

    mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_CTRL)
    
    update_state_from_joystick(state)
    m.light('front light').active = light_active

    mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_CTRL)

    with viewer.lock():
      viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)