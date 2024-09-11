from scipy.spatial.transform import Rotation
import numpy as np

# quat = [1, 0, 0, 0]
# rotation = Rotation.from_quat(quat)
# print(rotation.as_matrix())
# rotation = Rotation.from_matrix(np.dot(np.linalg.inv(np.array([[0, 1, 0],[-1, 0, 0],[0, 0, 1]])), rotation.as_matrix()))
# print(rotation.as_quat())


matrix = [[1, 0, 0],
          [0, -1, 0],
          [0, 0, -1]]
rotation = Rotation.from_matrix(matrix)
print(rotation.as_quat())

# from ur_ikfast import ur_kinematics
#
# ur3e_arm = ur_kinematics.URKinematics('ur3e')
#
# joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
# print("joint angles", joint_angles)
#
# pose_quat = ur3e_arm.forward(joint_angles)
# pose_matrix = ur3e_arm.forward(joint_angles, 'matrix')
#
# print("forward() quaternion \n", pose_quat)
# print("forward() matrix \n", pose_matrix)
#
# # print("inverse() all", ur3e_arm.inverse(pose_quat, True))
# print("inverse() one from quat", ur3e_arm.inverse(pose_quat, False, q_guess=joint_angles))
#
# print("inverse() one from matrix", ur3e_arm.inverse(pose_matrix, False, q_guess=joint_angles))