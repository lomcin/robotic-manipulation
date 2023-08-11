import time
import math

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('world/arm_ball.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  ball_r = 0.03
  ball_tolerance = ball_r*0.01
  rotation_period = math.pi*2.0/10.0
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

    # print('ball: ' + str(d.geom('ball').xpos))
    # print('ball_v: ' + str(d.sensor('ball_v').data))
    # print('shoulder: ' + str(d.joint('shoulder').qpos[0]))
    # print('elbow: ' + str(d.joint('elbow').qpos[0]))
    ball_p = d.geom('ball').xpos
    ball_v = d.efc_vel
    d.actuator('base_p').ctrl = (math.sin(d.time/rotation_period)*math.pi)
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