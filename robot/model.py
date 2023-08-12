import numpy as np

class RobotModel:

    def __init__(self, d, joint_names : list[str]):
        self.d = d
        self.qdimension = len(joint_names)
        self.qpos = np.zeros((self.qdimension), np.float64) # Generalized Robot Position
        self.ee = np.zeros((3), np.float64) # End Effector Cartesian Position
        self.jacobian = np.ndarray((), np.float64) # Jacobian Matrix, updated every step
        self.transform_ee = np.ndarray((4,4), np.float64) # Transform Matrix, updated every step
        self.update_jacobian = None
        self.update_transform = None
        self.joint_names = joint_names

    def set_update_jacobian_method(self, func):
        self.update_jacobian = func


    def set_update_transform_method(self, func):
        self.update_transform = func

    def update_ee(self):
        self.ee = self.transform_ee @ self.qpos

    def update(self):
        # self.update_jacobian(self)
        self.update_transform(self)
        # self.update_ee()

    def observe(self):
        for i, jname in enumerate(self.joint_names):
            self.qpos[i] = self.d.joint(jname).qpos[0]
    
    def actuate(self, actuator, value):
        self.d.actuator(actuator).ctrl = value

    def actuate_qpos(self, qpos):
        for i, jname in enumerate(self.joint_names):
            self.d.actuator(jname + '_p').ctrl = qpos[i]