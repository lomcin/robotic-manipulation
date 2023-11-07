import time
import math

# JOYSTICK
import pygame
pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print(f'joysticks:{joysticks}')

# ROS
import roslibpy
ros = roslibpy.Ros(host='localhost', port=9090)
ros.run()

# MUJOCO
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
sequential_id = 0

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
  # print(f'model:{dir(m)}')
  # print(f'data:{dir(d)}')
  # for i in range(0,m.njnt):
    # print(f'jnt:{m.jnt(i)}')
  tam = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_CTRL)
  state = np.zeros((tam,1),dtype=np.float64)

  clock = roslibpy.Topic(ros, '/clock', 'rosgraph_msgs/Clock')
  odom = roslibpy.Topic(ros, '/odom', 'nav_msgs/Odometry')

  while viewer.is_running() and ros.is_connected:
    step_start = time.time()

    mujoco.mj_step(m, d)

    mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_CTRL)

    xpos = d.body("car").xpos
    xquat = d.body("car").xquat
    # print(f"xpos:{xpos}")
    the_time = {'secs': int(np.floor(d.time)), 'nsecs': int(1000000000*(d.time-np.floor(d.time)))}
    the_header = {'seq': sequential_id, 'stamp': the_time, 'frame_id': 'odom'}
    sequential_id += 1
    the_position = {'x': xpos[0], 'y': xpos[1], 'z': xpos[2]}
    the_orientation = {'x': xquat[1], 'y': xquat[2], 'z': xquat[3], 'w':xquat[0]}

    the_pose = {'position': the_position, 'orientation': the_orientation}
    the_pose_with_covariance = {'pose': the_pose, 'covariance': list([0.0 for i in range(36)])}

    the_twist = {'linear':{'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular':{'x': 0.0, 'y': 0.0, 'z': 0.0}}
    the_twist_with_covariance = {'twist': the_twist, 'covariance': list([0.0 for i in range(36)])}

    odom_message = {'header': the_header, 'child_frame_id': '', 'pose': the_pose_with_covariance, 'twist': the_twist_with_covariance}

    clock.publish(roslibpy.Message({'clock': the_time}))
    odom.publish(roslibpy.Message(odom_message))
    
    update_state_from_joystick(state)
    m.light('front light').active = light_active

    mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_CTRL)

    with viewer.lock():
      viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
  
  if ros.is_connected: print("ROS is not connected.")
  else: ros.terminate()