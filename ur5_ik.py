import numpy as np


import ur5e_ikfast








class ur5_ik:
    def __init__(self):
        # Initialize kinematics for UR5 robot arm
        self.ur5_kin = ur5e_ikfast.PyKinematics()
        self.n_joints = self.ur5_kin.getDOF()

    def forward_kinematic(self, joint_angles):
        ee_pose = self.ur5_kin.forward(joint_angles)
        ee_pose = np.asarray(ee_pose).reshape(3, 4)  # 3x4 rigid transformation matrix
        return ee_pose

    def inverse_kinematic(self, ee_pose, cur_joint_angles):
        joint_configs = self.ur5_kin.inverse(ee_pose.reshape(-1).tolist())
        n_solutions = int(len(joint_configs) / self.n_joints)
        if n_solutions == 0:
            print('no solution found')
            return None
        joint_configs = np.asarray(joint_configs).reshape(n_solutions, self.n_joints)
        #  fix multi-solves problem
        move_ranges = []
        for joint_config in joint_configs:
            move_range = 0
            for n in range(self.n_joints):
                move_range += abs(joint_config[n]-cur_joint_angles[n])
            move_ranges.append(move_range)
        min_val = min(move_ranges)
        min_index = move_ranges.index(min_val)
        return joint_configs[min_index]


# ur5_module = ur5_ik()
# print(ur5_module.forward_kinematic([-2.71316499e-01,  8.07371497e-01, -1.87837207e+00 , 1.07100058e+00
#  ,-2.71316499e-01 ,-3.55271368e-15]))





