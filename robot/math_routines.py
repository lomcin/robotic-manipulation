import math
import numpy as np

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

def ee_from_qpos(qpos):
    v = np.array([0, 0, 0, 1])
    base_m = transform_matrix(qpos[0], axis=2, t=[0, 0, 0])
    shoulder_m = transform_matrix(qpos[1], axis=1, t=[0, 0, 0.045])
    elbow_m = transform_matrix(qpos[2], axis=1, t=[0, 0, 0.5])
    fake_hand_m = transform_matrix(0, axis=1, t=[0, 0, 0.5])
    transform_ee = base_m @ shoulder_m @ elbow_m @ fake_hand_m
    return transform_ee @ v.transpose(), transform_ee

def update_transform_method(r):
    ee, t_ee = ee_from_qpos(r.qpos)
    r.transform_ee = t_ee
    r.ee = ee