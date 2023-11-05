import time
import math

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

m = mujoco.MjModel.from_xml_path('world/base.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  print(f'model:{dir(m)}')
  print(f'data:{dir(d)}')
  print(f'data.jnt("ball"):{d.jnt("ball")}')
  print(f'data.joint("ball"):{d.joint("ball")}')
  for i in range(0,m.njnt):
    print(f'jnt:{m.jnt(i)}')

  while viewer.is_running():
  # while True:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)
   
    # print('ball: ' + str(d.geom('ball').xpos))
    # print('ball_v: ' + str(d.sensor('ball_v').data))

    tam = mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_QPOS)
    state = np.zeros((tam,1),dtype=np.float64)
    mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_QPOS)
    # print(f"state: {state}")
    state[0] = d.time*0.1
    mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_QPOS)
    # print(f"d.geom('ball').xpos:{d.geom('ball').xpos}")
    with viewer.lock():
      viewer.sync()
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)