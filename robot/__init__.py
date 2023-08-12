from .math_routines import *
from .model import *
from .task_scheduler import *
from .ball_catcher_tasks import *

if __name__ == "__main__":
    robot = RobotModel(3)
    print(robot.jacobian)