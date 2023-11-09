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

def create_the_time(d):
  the_time = {'secs': int(np.floor(d.time)), 'nsecs': int(1000000000*(d.time-np.floor(d.time)))}
  return the_time

def create_clock_message(the_time):
  return {'clock': the_time}

def create_odom_message(the_time, d, body_name):
  global sequential_id

  xpos = d.body(body_name).xpos
  xquat = d.body(body_name).xquat

  the_header = {'seq': sequential_id, 'stamp': the_time, 'frame_id': 'world'}
  the_position = {'x': xpos[0], 'y': xpos[1], 'z': xpos[2]}
  the_orientation = {'x': xquat[1], 'y': xquat[2], 'z': xquat[3], 'w':xquat[0]}

  the_pose = {'position': the_position, 'orientation': the_orientation}
  the_pose_with_covariance = {'pose': the_pose, 'covariance': list([0.0 for i in range(36)])}

  the_twist = {'linear':{'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular':{'x': 0.0, 'y': 0.0, 'z': 0.0}}
  the_twist_with_covariance = {'twist': the_twist, 'covariance': list([0.0 for i in range(36)])}

  odom_message = {'header': the_header, 'child_frame_id': 'base_link', 'pose': the_pose_with_covariance, 'twist': the_twist_with_covariance}
  return odom_message

def create_tf_world_message(the_time):
  global sequential_id

  xpos = [0.0, 0.0, 0.0]
  xquat = [1.0, 0.0, 0.0, 0.0]

  the_header = {'seq': sequential_id, 'stamp': the_time, 'frame_id': 'world'}
  the_translation = {'x': xpos[0], 'y': xpos[1], 'z': xpos[2]}
  the_rotation = {'x': xquat[1], 'y': xquat[2], 'z': xquat[3], 'w':xquat[0]}

  the_transform = {'translation': the_translation, 'rotation': the_rotation}

  tf_message = {'header': the_header, 'child_frame_id': '', 'transform': the_transform}
  return tf_message

def create_tf_message(the_time, d, body_name):
  global sequential_id

  xpos = d.body(body_name).xpos
  xquat = d.body(body_name).xquat

  the_header = {'seq': sequential_id, 'stamp': the_time, 'frame_id': 'world'}
  the_translation = {'x': xpos[0], 'y': xpos[1], 'z': xpos[2]}
  the_rotation = {'x': xquat[1], 'y': xquat[2], 'z': xquat[3], 'w':xquat[0]}

  the_transform = {'translation': the_translation, 'rotation': the_rotation}

  tf_message = {'header': the_header, 'child_frame_id': 'base_link', 'transform': the_transform}
  return tf_message

def create_tf2_message(the_time, d, bodies_names):
  global sequential_id

  # tf_world_message = create_tf_world_message(the_time)

  transforms = list([])

  for body_name in bodies_names:
    transforms.append(create_tf_message(the_time, d, body_name))

  tf_message = {'transforms': transforms}

  return tf_message

def create_joint_states_message(the_time, m, d, body_name):
  global sequential_id

  tam_qpos = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_QPOS)
  state_qpos = np.zeros((tam_qpos,1),dtype=np.float64)
  tam_qvel = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_QVEL)
  state_qvel = np.zeros((tam_qvel,1),dtype=np.float64)
  tam_qfrc = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_QFRC_APPLIED)
  state_qfrc = np.zeros((tam_qfrc,1),dtype=np.float64)

  mujoco.mj_getState(m,d,state_qpos,mujoco.mjtState.mjSTATE_QPOS)
  mujoco.mj_getState(m,d,state_qvel,mujoco.mjtState.mjSTATE_QVEL)
  mujoco.mj_getState(m,d,state_qfrc,mujoco.mjtState.mjSTATE_QFRC_APPLIED)

  names = list()
  qids = list()
  qdofids = list()

  # print(f'm:{dir(m)}')

  
  for i in range(0,m.njnt):
    jnt = m.joint(i)
    # print(f'jnt:{jnt}')
    if (jnt.type[0] == 3): # hinge joint
      names.append(jnt.name)
      qids.append(jnt.qposadr[0])
      qdofids.append(jnt.dofadr[0])

  # print(f'qpos: {tam_qpos}, qvel: {tam_qvel}, qfrc: {tam_qfrc} names: {names}, qids: {qids}, qdofids: {qdofids}')
  position = state_qpos[qids].flatten().tolist()
  velocity = state_qvel[qdofids].flatten().tolist()
  effort = state_qfrc[qdofids].flatten().tolist()

  # print(f'position:{position}, velocity:{velocity}, effort:{effort}, ')
  
  the_header = {'seq': sequential_id, 'stamp': the_time, 'frame_id': 'base_link'}
  return {'header': the_header, 'name': names, 'position': position, 'velocity': velocity, 'effort': effort}


with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  # print(f'model:{dir(m)}')
  # print(f'data:{dir(d)}')
  # for i in range(0,m.njnt):
    # print(f'jnt:{m.jnt(i)}')

  

  tam_ctrl = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_CTRL)
  state_ctrl = np.zeros((tam_ctrl,1),dtype=np.float64)

  topic_clock = roslibpy.Topic(ros, '/clock', 'rosgraph_msgs/Clock')
  topic_odom = roslibpy.Topic(ros, '/odom', 'nav_msgs/Odometry')
  topic_tf = roslibpy.Topic(ros, '/tf', 'tf2_msgs/TFMessage')
  topic_joint_states = roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState')

  # Reset clock with negative value
  # clock_message = {'clock':{'secs':-1, 'nsecs':0}}
  # topic_clock.publish(roslibpy.Message(clock_message))
  # tf_world_message = create_tf_world_message({'secs':-1, 'nsecs':0})
  # topic_tf.publish(roslibpy.Message(tf_world_message))

  while viewer.is_running() and ros.is_connected:
    step_start = time.time()

    mujoco.mj_step(m, d)

    mujoco.mj_getState(m,d,state_ctrl,mujoco.mjtState.mjSTATE_CTRL)

    the_time = create_the_time(d)

    # Clock
    clock_message = create_clock_message(the_time)
    topic_clock.publish(roslibpy.Message(clock_message))

    # Odometry
    odom_message = create_odom_message(the_time, d, "car")
    topic_odom.publish(roslibpy.Message(odom_message))

    # Joint States
    joint_states_message = create_joint_states_message(the_time, m, d, "car")
    topic_joint_states.publish(roslibpy.Message(joint_states_message))


    # Transforms
    bodies_names = ['car']
    tf2_message = create_tf2_message(the_time, d, bodies_names)
    topic_tf.publish(roslibpy.Message(tf2_message))
    
    update_state_from_joystick(state_ctrl)
    m.light('front light').active = light_active

    mujoco.mj_setState(m,d,state_ctrl,mujoco.mjtState.mjSTATE_CTRL)

    with viewer.lock():
      viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
    
    sequential_id += 1

  
  if ros.is_connected: print("ROS is not connected.")
  else: ros.terminate()