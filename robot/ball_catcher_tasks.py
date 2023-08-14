from .math_routines import *
from .model import *
from .task_scheduler import *



# go_to_ball
task_go_to_ball = RobotTask("go_to_ball")
def check_completion_go_to_ball(rt: RobotTask):
    rt.done = cartesian_distance(rt.target, rt.robot.ee[:-1]) < 0.1

def actuate_go_to_ball(rt: RobotTask):
    bqpos = optim_qpos_from_xpos(rt.target, rt.robot.qpos)
    rt.robot.actuate_qpos(bqpos)

task_go_to_ball.set_check_completion_method(check_completion_go_to_ball)
task_go_to_ball.set_actuate_method(actuate_go_to_ball)


# grab_ball
task_grab_ball = RobotTask("grab_ball")
def check_completion_grab_ball(rt: RobotTask):
    rt.done = (rt.target - rt.begin_time) > 1

def actuate_grab_ball(rt: RobotTask):
    rt.robot.actuate('fake_hand_grab', 300)
    if rt.begin_time is None:
        rt.begin_time = rt.target

task_grab_ball.set_check_completion_method(check_completion_grab_ball)
task_grab_ball.set_actuate_method(actuate_grab_ball)


# move_ball_to_destination
task_move_ball_to_destination = RobotTask("move_ball_to_destination")
def check_completion_move_ball_to_destination(rt: RobotTask):
    rt.done = cartesian_distance(rt.target, rt.robot.ee[:-1]) < 0.1

def actuate_move_ball_to_destination(rt: RobotTask):
    bqpos = optim_qpos_from_xpos(rt.target, rt.robot.qpos, 1)
    rt.robot.actuate_qpos(bqpos)

task_move_ball_to_destination.set_check_completion_method(check_completion_move_ball_to_destination)
task_move_ball_to_destination.set_actuate_method(actuate_move_ball_to_destination)


# ungrab_ball
task_ungrab_ball = RobotTask("ungrab_ball")
def check_completion_ungrab_ball(rt: RobotTask):
    pass

def actuate_ungrab_ball(rt: RobotTask):
    rt.robot.actuate('fake_hand_grab', 0)
    rt.done = True

task_ungrab_ball.set_check_completion_method(check_completion_ungrab_ball)
task_ungrab_ball.set_actuate_method(actuate_ungrab_ball)