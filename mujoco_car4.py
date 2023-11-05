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
a = 1

def update_state_from_joystick(state):
  for event in pygame.event.get():
    if event.type == pygame.JOYAXISMOTION:
      cmd_accelerate = joysticks[0].get_axis(5) > acc_threshold
      x_accelerate = (joysticks[0].get_axis(5) - acc_threshold)/(cmd_acc_norm)
      cmd_break = joysticks[0].get_axis(4) > acc_threshold
      x_break = (joysticks[0].get_axis(5) - acc_threshold)/cmd_acc_norm

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


with mujoco.viewer.launch_passive(m, d) as viewer:
  # viewer.opt.flags[mujoco.mjtVisFlag.mjOPT_FULLSCREEN] = True
  print(f'mujoco.mjtVisFlag:{dir(mujoco.mjtVisFlag)}')
# with a as aa:
# while True:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  ball_r = 0.03
  ball_tolerance = ball_r*0.01
  rotation_period = math.pi*2.0/10.0
  print(f'model:{dir(m)}')
  print(f'data:{dir(d)}')
  # print(f'data.jnt("ball"):{d.jnt("ball")}')
  # print(f'data.joint("ball"):{d.joint("ball")}')
  for i in range(0,m.njnt):
    print(f'jnt:{m.jnt(i)}')

  while viewer.is_running():
  # while True:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    # viewer.sync()

    # print('ball: ' + str(d.geom('ball').xpos))
    # print('ball_v: ' + str(d.sensor('ball_v').data))
    # print('shoulder: ' + str(d.joint('shoulder').qpos[0]))
    # print('elbow: ' + str(d.joint('elbow').qpos[0]))
    tam = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_CTRL)
    state = np.zeros((tam,1),dtype=np.float64)
    # ball_p = d.geom('ball').xpos
    # ball_p[0] = d.time
    # d.geom('ball').xpos = ball_p
    # m.body_pos[0][0] = d.time*0.1
    mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_CTRL)
    # print(f"state: {state}")
    # state[0] = d.time*0.1

    update_state_from_joystick(state)
    mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_CTRL)
    # print(f"d.geom('ball').xpos:{d.geom('ball').xpos}")
    # d.actuator('base_p').ctrl = (math.sin(d.time/rotation_period)*math.pi)
    with viewer.lock():
      viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)