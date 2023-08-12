import time
import math
import robot
import numpy as np

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('world/arm_ball_catcher.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  ball_r = 0.03
  ball_tolerance = ball_r*0.01
  rotation_period = math.pi*2.0/10.0
  

  joint_names = ["base", "shoulder", "elbow"]
  arm = robot.RobotModel(d, joint_names)
  ball_p = np.zeros(3)
  ball_v = np.zeros(3)

  # Set End Effector Estimator Function
  arm.set_update_transform_method(robot.update_transform_method)

  while viewer.is_running():
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Update robot model observation
    arm.observe()

    # print(f'arm qpos: {arm.qpos}')
    ball_p = d.geom('ball').xpos
    ball_v = d.sensor('ball_vx').data
    # d.actuator('base_p').ctrl = (math.sin(d.time/rotation_period)*math.pi)

    # Update arm End Effector
    arm.update()

    print(f'arm ee: {arm.ee} and qpos {arm.qpos}')
    
    # Actuate testing
    # base_p = (math.sin(d.time/rotation_period)*math.pi)
    # arm.actuate('base_p', base_p)

    with viewer.lock():
      if abs(ball_p[2] - ball_r) < ball_tolerance:
        # ball_p[2] = 1
        # ball_v[2] = 0
        # print('ballafter: ' + str(d.geom('ball').xpos))
        # d.geom('ball').xpos = ball_p
        # d.efc_vel = ball_v
        viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)