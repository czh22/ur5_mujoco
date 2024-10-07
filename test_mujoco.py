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
import force_feedback

# 设置admittance控制参数
M = np.array([[20, 0, 0, 0, 0, 0],
              [0, 20, 0, 0, 0, 0],
              [0, 0, 20, 0, 0, 0],
              [0, 0, 0, 20, 0, 0],
              [0, 0, 0, 0, 20, 0],
              [0, 0, 0, 0, 0, 20]])
B = np.array([[200, 0, 0, 0, 0, 0],
              [0, 200, 0, 0, 0, 0],
              [0, 0, 200, 0, 0, 0],
              [0, 0, 0, 200, 0, 0],
              [0, 0, 0, 0, 200, 0],
              [0, 0, 0, 0, 0, 200]])
K = np.array([[180, 0, 0, 0, 0, 0],
              [0, 180, 0, 0, 0, 0],
              [0, 0, 180, 0, 0, 0],
              [0, 0, 0, 180, 0, 0],
              [0, 0, 0, 0, 180, 0],
              [0, 0, 0, 0, 0, 180]])
current_vel = np.array([0, 0, 0, 0, 0, 0])
current_pos = np.array([0, 0, 0, 0, 0, 0])


communicator = force_feedback.SerialCommunicator('/dev/ttyUSB0', 9600, send_frequency=4)

## 初始化imu接收器
rot_receiver = imu_receiver.imu_receiver('/dev/ttyUSB1', 921600, 20)
## 初始化相机位置获取
cam2pos = camera2pos.cam2pos()
## 初始化ur5逆解
ur5e_arm = ur_kinematics.URKinematics('ur5e')

init_imu = rot_receiver.get_data()
init_quat = init_imu[6:10]
init_matrix = Rotation.from_quat(init_quat).as_matrix()

# model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/ur5e.xml')
model = mujoco.MjModel.from_xml_path('demo.xml')
# model.opt.gravity = 0
data = mujoco.MjData(model)
## 横着
# rotation = np.array([[-1, 0, 0],
#                      [0, 0, 1],
#                      [0, 1, 0]])
## 朝下
rotation = np.array([[-1.64292755e-03,  9.98002831e-01, -6.31478444e-02],
 [ 9.99998335e-01,  1.58947619e-03, -8.96674718e-04],
 [-7.94511912e-04, -6.31492124e-02, -9.98003780e-01]]
)
trans_matrix = np.dot(rotation, np.linalg.inv(init_matrix))


## 主循环
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    real_start_time = time.time()
    while viewer.is_running():

        start_time = time.time()
## 获取imu rotation
        imu_data = rot_receiver.get_data()
        quat = imu_data[6:10]
        rotation = Rotation.from_quat(quat).as_matrix()
        rotation = np.dot(trans_matrix, rotation)
   
        
## 视频释教的位置获取
        position = cam2pos.get_pos()

        ## 获取末端受力和力矩
        force = data.sensor('force_sensor').data
        torque = data.sensor('torque_sensor').data

                        ## 获取传感器的位置和旋转矩阵
        sensor_pos = data.site('attachment_site').xpos
        sensor_mat = data.site('attachment_site').xmat.reshape(3, 3)

        ## 将力转换到世界坐标系
        force_world = sensor_mat.dot(force)

        ## 将力矩转换到世界坐标系
        torque_world = sensor_mat.dot(torque)

        ## 合并力和力矩
        end_force = np.concatenate((force_world, torque_world), axis=0)

        ## 计算feedback_force
        feedback_force = np.sqrt(np.sum(np.square(force_world)))

        print("反馈力的平方和：", feedback_force)
## 发送力给arduino
        communicator.set_value(feedback_force)

## 进行位置补偿
        position[0] += 0.3
        position[2] += 0
        print(position)
            
        pose = np.concatenate((rotation, np.array(position)[:, np.newaxis]), axis=1)

        # 示教姿态3x4转6
        pose_rotation = Rotation.from_matrix(pose[:, :3])
        euler_angles = pose_rotation.as_euler('xyz')
        target_pos = np.concatenate((pose[:, -1], euler_angles), axis=0)

            # 限制力的大小
        for i in range(len(end_force)):
            if end_force[i] > 50:
                end_force[i] = 50
            elif end_force[i] < -50:
                end_force[i] = -50
                ## 计算admittance控制量
        current_pos, current_vel = admittance_py.admittance_pos(K, B, M, current_vel, current_pos, end_force, target_pos)

        pose_rotation = Rotation.from_euler('xyz', current_pos[3:])
        matrix = pose_rotation.as_matrix()
        target_pos = np.concatenate((matrix, current_pos[:3, np.newaxis]), axis=1)
## 求解逆运动学               
        ctrl = ur5e_arm.inverse(target_pos, False, data.qpos[-6:])
                
## 设置控制量
        if ctrl is not None:
            print(True)
            data.ctrl[-6:] = ctrl
## 进行仿真
        current_time = time.time()
        ## 确保仿真时间和真实时间同步
        if current_time - real_start_time < data.time:
            time.sleep(data.time - (current_time - real_start_time))
        mujoco.mj_step(model, data, 10)
        viewer.sync()

        
