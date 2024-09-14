import mujoco
import mujoco.viewer
import time

import camera2pos
import numpy as np
from scipy.spatial.transform import Rotation
import math
import admittance_py
from ur_ikfast import ur_kinematics
import imu_receiver

# 设置admittance控制参数
M = np.array([[5, 0, 0, 0, 0, 0],
     [0, 5, 0, 0, 0, 0],
     [0, 0, 5, 0, 0, 0],
     [0, 0, 0, 5, 0, 0],
     [0, 0, 0, 0, 5, 0],
     [0, 0, 0, 0, 0, 5]])
B = np.array([[5, 0, 0, 0, 0, 0],
     [0, 5, 0, 0, 0, 0],
     [0, 0, 5, 0, 0, 0],
     [0, 0, 0, 5, 0, 0],
     [0, 0, 0, 0, 5, 0],
     [0, 0, 0, 0, 0, 5]])
K = np.array([[10, 0, 0, 0, 0, 0],
     [0, 10, 0, 0, 0, 0],
     [0, 0, 10, 0, 0, 0],
     [0, 0, 0, 10, 0, 0],
     [0, 0, 0, 0, 10, 0],
     [0, 0, 0, 0, 0, 10]])
current_vel = np.array([0, 0, 0, 0, 0, 0])
current_pos = np.array([0, 0, 0, 0, 0, 0])


# 初始化ur5逆解
ur5e_arm = ur_kinematics.URKinematics('ur5e')
# ur5_module = ur5_ik.ur5_ik()

# model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/ur5e.xml')
model = mujoco.MjModel.from_xml_path('demo.xml')
# model.opt.gravity = 0
data = mujoco.MjData(model)
# 横着
# rotation = np.array([[-1, 0, 0],
#                      [0, 0, 1],
#                      [0, 1, 0]])
# 朝下
rotation = np.array([[-1.64292755e-03,  9.98002831e-01, -6.31478444e-02],
 [ 9.99998335e-01,  1.58947619e-03, -8.96674718e-04],
 [-7.94511912e-04, -6.31492124e-02, -9.98003780e-01]]
)


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        position = [-0.75, 0.1, 0.2]
        start_time = time.time()


        if position is not None:

            pose = np.concatenate((rotation, np.array(position)[:, np.newaxis]), axis=1)
            # print(pose)
            # 求逆解
            ctrl = ur5e_arm.inverse(pose, False, [-3.14, -0.251, 0.0943, -1.51, -1.57, 0])
            # print(ctrl)

            if ctrl is not None:

                # 释教姿态3x4转6
                pose_rotation = Rotation.from_matrix(pose[:, :3])
                euler_angles = pose_rotation.as_euler('xyz')
                target_pos = np.concatenate((pose[:, -1], euler_angles), axis=0)

                # end_pose = np.concatenate(
                #     (data.site('attachment_site').xmat.reshape(3, 3), data.site('attachment_site').xpos[:, np.newaxis]), axis=1)
                # print(end_pose)
 
                # end_rotation = Rotation.from_matrix(end_pose[:, :-1])
                # end_euler_angles = end_rotation.as_euler('xyz')
                # current_pos = np.concatenate((end_pose[:, -1], end_euler_angles), axis=0)
# 获取末端受力和力矩
                force = data.sensor('force_sensor').data
                torque = data.sensor('torque_sensor').data
                end_force = np.concatenate((force, torque), axis=0)
                # end_force = np.array([0, 0, 0, 0, 0, 0])
                # print(end_force)
# 计算admittance控制量
                current_pos, current_vel = admittance_py.admittance_pos(K, B, M, current_vel, current_pos, end_force, target_pos)

                pose_rotation = Rotation.from_euler('xyz', current_pos[3:])
                matrix = pose_rotation.as_matrix()
                target_pos = np.concatenate((matrix, current_pos[:3, np.newaxis]), axis=1)
                # print(target_pos)
                # print(data.qpos.shape)
                # ctrl = ur5e_arm.inverse(target_pos, False, data.qpos[-6:])
                # print(ctrl)
# 设置控制量
                if ctrl is not None:
                    print(True)
                    data.ctrl[-6:] = ctrl
# 进行仿真
        mujoco.mj_step(model, data, 1)
        viewer.sync()

        # print(time.time() - start_time)
