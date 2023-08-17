import time
import math
import robot
import numpy as np

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('world/arm_ball_catcher.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()

  # Ball information
  ball_r = 0.03
  ball_tolerance = ball_r*0.01
  ball_p = np.zeros(3)
  ball_v = np.zeros(3)

  ball_destination_p = np.array([0.5, -0.5, 0.15])

  # Setting up the Arm Robot
  joint_names = ["base", "shoulder", "elbow"]
  arm = robot.RobotModel(d, joint_names)

  # Set End Effector Estimator Function
  arm.set_update_transform_method(robot.update_transform_method)

  # Setting up the task scheduler
  ts = robot.task_scheduler.RobotTaskScheduler()

  # go_to_ball
  robot.ball_catcher_tasks.task_go_to_ball.robot = arm
  ts.add(robot.ball_catcher_tasks.task_go_to_ball)

  # grab_ball
  robot.ball_catcher_tasks.task_grab_ball.robot = arm
  ts.add(robot.ball_catcher_tasks.task_grab_ball)

  # move_ball_to_destination
  robot.ball_catcher_tasks.task_move_ball_to_destination.robot = arm
  ts.add(robot.ball_catcher_tasks.task_move_ball_to_destination)

  # ungrab_ball
  robot.ball_catcher_tasks.task_ungrab_ball.robot = arm
  ts.add(robot.ball_catcher_tasks.task_ungrab_ball)


  while viewer.is_running():
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Update robot model observation
    arm.observe()

    ball_p = d.geom('ball').xpos
    ball_v = d.sensor('ball_vx').data
    
    # Update arm End Effector
    arm.update()

    # Actuate using the task scheduler
    if not ts.empty():
      if ts.current_task().name == "go_to_ball":
        ts.current_task().target = ball_p
      elif ts.current_task().name == "grab_ball":
        ts.current_task().target = float(d.time)
      elif ts.current_task().name == "move_ball_to_destination":
        ts.current_task().target = ball_destination_p
    ts.spin_once(d.time)

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)