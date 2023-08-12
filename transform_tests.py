import math
import robot
import numpy as np
joint_names = ["base", "shoulder", "elbow"]
d = None
arm = robot.RobotModel(d, joint_names)
arm.qpos = [0, math.pi*0.5, 0]
# arm.qpos = [0, 0, math.pi*0.5]

def update_transform_method(r):
    v = np.array([0, 0, 0, 1])
    base_m = robot.transform_matrix(r.qpos[0], axis=2, t=[0, 0, 0])
    shoulder_m = robot.transform_matrix(r.qpos[1], axis=1, t=[0, 0, 0.045])
    elbow_m = robot.transform_matrix(r.qpos[2], axis=1, t=[0, 0, 0.5])
    fake_hand_m = robot.transform_matrix(0, axis=1, t=[0, 0, 0.5])
    r.transform_ee = base_m @ shoulder_m @ elbow_m @ fake_hand_m
    print(f'base_m {base_m}')
    print(f'shoulder_m {shoulder_m}')
    print(f'elbow_m {elbow_m}')
    print(f'transform_ee {r.transform_ee}')
    r.ee = r.transform_ee @ v.transpose()


update_transform_method(arm)
print(f'arm.ee: {arm.ee} arm.qpos: {arm.qpos}')
print('')
qpos = robot.optim_qpos_from_xpos([0.5, 0, 0], arm.qpos)
ee, t_ee = robot.ee_from_qpos(qpos)
print(f'qpos: {qpos} ee: {ee}')