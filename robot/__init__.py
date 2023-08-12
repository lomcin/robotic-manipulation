from .model import *
import math

def rot_matrix(ang, axis=0):
    c = math.cos(ang)
    s = math.sin(ang)
    if axis == 0: # X-axis
        return np.array([[1, 0, 0]],
                         [0, c, -s],
                         [0, s, c])
    elif axis == 1: # Y-axis
        return np.array([[c, 0, s],
                         [0, 1, 0],
                         [-s, 0, c]])
    elif axis == 2: # Z-axis
        return np.array([[c, -s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
    else:
        raise Exception("Invalid axis. Use axis = [0, 1, 2]")
    
def transform_matrix(ang, axis=0, t=[0,0,0]):
    c = math.cos(ang)
    s = math.sin(ang)
    if axis == 0: # X-axis
        return np.array([[1, 0, 0, t[0]],
                         [0, c, -s, t[1]],
                         [0, s, c, t[2]],
                         [0, 0, 0, 1]])
    elif axis == 1: # Y-axis
        return np.array([[c, 0, s, t[0]],
                         [0, 1, 0, t[1]],
                         [-s, 0, c, t[2]],
                         [0, 0, 0, 1]])
    elif axis == 2: # Z-axis
        return np.array([[c, -s, 0, t[0]],
                         [s, c, 0, t[1]],
                         [0, 0, 1, t[2]],
                         [0, 0, 0, 1]])
    else:
        raise Exception("Invalid axis. Use axis = [0, 1, 2]")

if __name__ == "__main__":
    robot = RobotModel(3)
    print(robot.jacobian)