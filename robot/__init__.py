from .math_routines import *
from .model import *

if __name__ == "__main__":
    robot = RobotModel(3)
    print(robot.jacobian)